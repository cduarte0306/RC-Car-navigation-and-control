#include "VideoLaneNet.hpp"

#include "utils/logger.hpp"

#include <fstream>
#include <filesystem>
#include <cuda_runtime_api.h>


using namespace nvonnxparser;


class TrtLogger : public nvinfer1::ILogger {
public:
    TrtLogger() = default;
    explicit TrtLogger(const char* name) : nvinfer1::ILogger(), m_Name(name) {}
    void log(Severity severity, const char* msg) noexcept override {
        // Filter out info-level noise, only print warnings and above
        if (severity <= Severity::kWARNING) {
            Logger* logger = Logger::getLoggerInst();
            logger->log(Logger::LOG_LVL_ERROR, "[TensorRT] %s\r\n", msg);
        }
    }

private:
    const char* m_Name = nullptr;
};


static TrtLogger trtLogger("LaneNet");
static cudaStream_t inferenceStream;


namespace Vision {
    LaneNet::LaneNet(const char* onnxModelPath) : m_OnnxPath(onnxModelPath), m_LaneMaskOut() {}

    LaneNet::LaneNet(const char* onnxModelPath, const char* enginePath)
        : m_OnnxPath(onnxModelPath), m_EnginePath(enginePath), m_LaneMaskOut() {}

    LaneNet::~LaneNet() {
        cudaFree(m_InputBuffer);
        cudaFree(m_OutputBuffer);
        cudaStreamDestroy(inferenceStream);
        delete m_Context;
        delete m_Engine;
        delete m_Runtime;

    }

    int LaneNet::init() {
        Logger* logger = Logger::getLoggerInst();

        // Resolve engine path: explicit override takes priority
        std::filesystem::path enginePath = m_EnginePath
            ? std::filesystem::path(m_EnginePath)
            : std::filesystem::path(m_OnnxPath).replace_extension(".engine");

        if (!std::filesystem::exists(enginePath)) {
            logger->log(Logger::LOG_LVL_INFO, "No engine file found at %s. Generating...\r\n", enginePath.string().c_str());
            onnxToEngine(m_OnnxPath, enginePath.string().c_str());
        }

        // Load the engine and prepare for inference
        m_Runtime = nvinfer1::createInferRuntime(trtLogger);

        std::vector<char> modelData;
        std::ifstream engineFile(enginePath, std::ios::binary);
        if (!engineFile) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to open engine file: %s\r\n", enginePath.string().c_str());
            return -1; // Return -1 on failure
        }

        modelData = std::vector<char>((std::istreambuf_iterator<char>(engineFile)), std::istreambuf_iterator<char>());
        engineFile.close();

        m_Engine = m_Runtime->deserializeCudaEngine(modelData.data(), modelData.size());
        if (!m_Engine) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to deserialize engine from file: %s\r\n", enginePath.string().c_str());
            return -1; // Return -1 on failure
        }

        m_Context = m_Engine->createExecutionContext();
        if (!m_Context) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to create execution context from engine\r\n");
            return -1; // Return -1 on failure
        }

        const char* inputName = m_Engine->getIOTensorName(0);
        m_TensorWidth  = m_Engine->getTensorShape(inputName).d[3]; // NCHW
        m_TensorHeight = m_Engine->getTensorShape(inputName).d[2];

        logger->log(Logger::LOG_LVL_INFO, "Tensor defined with width: %d, height: %d\r\n", m_TensorWidth, m_TensorHeight);
        
        // Allocate cuda input and output buffers
        cudaMalloc(&m_InputBuffer, 1 * 3 * m_TensorWidth * m_TensorHeight * sizeof(float)); // Assuming 3 channels (RGB)
        cudaMalloc(&m_OutputBuffer, 1 * 1 * m_TensorWidth * m_TensorHeight * sizeof(float)); // Assuming 1 channel (lane mask)
        
        m_Context->setInputTensorAddress(INPUT_TENSOR_NAME, m_InputBuffer);
        m_Context->setOutputTensorAddress(OUTPUT_TENSOR_NAME, m_OutputBuffer); // Set output buffer address later during inference
        m_Context->setInputShape(INPUT_TENSOR_NAME, nvinfer1::Dims4(1, 3, m_TensorHeight, m_TensorWidth)); // Set input shape (NCHW)   
        cudaStreamCreate(&inferenceStream);

        // Allocate lane mask output buffer
        m_LaneMaskOut = cv::Mat(m_TensorHeight, m_TensorWidth, CV_32FC1); // Assuming single-channel output for lane mask
        m_D_LaneMaskOut.create(m_TensorHeight, m_TensorWidth, CV_32FC1);
        return 0; // Return 0 on success
    }

    int LaneNet::onnxToEngine(const char* onnxModelPath, const char* enginePath) {
        using namespace nvonnxparser;

        Logger* logger = Logger::getLoggerInst();
        nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(trtLogger);
        nvinfer1::INetworkDefinition* network = builder->createNetworkV2(1 << static_cast<int>(
            nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
        IParser* parser = nvonnxparser::createParser(*network, trtLogger);
        parser->parseFromFile(m_OnnxPath, static_cast<int>(nvinfer1::ILogger::Severity::kWARNING));
        bool failure = false; // Assume success unless we find errors
        for (int i = 0; i < parser->getNbErrors(); ++i) {
            auto des = parser->getError(i)->desc();            
            logger->log(Logger::LOG_LVL_ERROR, "ONNX Parser error: %s\r\n", des);
            failure = true;
        }

        if (failure) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to parse ONNX model. Aborting engine creation.\r\n");
            return -1; // Return -1 on failure
        }

        logger->log(Logger::LOG_LVL_INFO, "Successfully parsed ONNX model. Building TensorRT engine...\r\n");
        nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1U << 30); // 1GB workspace
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        nvinfer1::IHostMemory* serializedModel = builder->buildSerializedNetwork(*network, *config);

        if (!serializedModel) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to build TensorRT engine. Aborting.\r\n");
            return -1; // Return -1 on failure
        }

        // Save the serialized engine to a file
        std::ofstream engineFile(std::string(enginePath), std::ios::binary);
        if (!engineFile) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to open engine file for writing: %s\r\n", enginePath);
            return -1; // Return -1 on failure
        }
        engineFile.write(reinterpret_cast<const char*>(serializedModel->data()), serializedModel->size());
        engineFile.close();
        logger->log(Logger::LOG_LVL_INFO, "Successfully built and saved TensorRT engine to %s\r\n", enginePath);

        // Engine file built, clean up
        delete parser;
        delete network;
        delete config;
        delete builder;

        return 0; // Return 0 on success
    }

    int LaneNet::infer(const cv::Mat& inputFrame, cv::Mat& outputFrame) {
        Logger* logger = Logger::getLoggerInst();
        // Resize input frame to match model input dimensions
        cv::cuda::Stream stream;
        cv::Mat mask;
        cv::cuda::GpuMat d_frame(inputFrame);
        cv::cuda::GpuMat d_resized, d_normalized, d_maskColor;
        cv::cuda::resize(d_frame, d_resized, cv::Size(m_TensorWidth, m_TensorHeight), 0, 0, cv::INTER_LINEAR, stream);
        cv::cuda::GpuMat d_rgb;
        cv::cuda::cvtColor(d_resized, d_rgb, cv::COLOR_BGR2RGB, 0, stream); // Model expects RGB

        d_rgb.convertTo(d_normalized, CV_32F, 1.0 / 255, 0, stream); // Normalize to [0, 1]

        // Execute inference (using the execution context and buffers set up in init())
        cv::cuda::GpuMat d_channels[3];
        cv::cuda::split(d_normalized, d_channels, stream); // Split into separate channels for direct copying to input buffer

        for (int i  = 0; i < 3; ++i) {
            // Normalize each channel using ImageNet mean and std
            cv::cuda::subtract(d_channels[i], MEAN[i], d_channels[i], cv::noArray(), -1, stream);
            cv::cuda::divide(d_channels[i], STD[i], d_channels[i], 1, -1, stream);
        }

        stream.waitForCompletion(); // Ensure all preprocessing is done before copying to input buffer

        float* dst = static_cast<float*>(m_InputBuffer);
        size_t rowBytes = m_TensorWidth * sizeof(float);
        for (int c = 0; c < 3; ++c) {
            cudaMemcpy2DAsync(dst + c * m_TensorHeight * m_TensorWidth,
                              rowBytes,                  // dst pitch (contiguous)
                              d_channels[c].data,
                              d_channels[c].step,        // src pitch (may be padded)
                              rowBytes,                  // width in bytes
                              m_TensorHeight,
                              cudaMemcpyDeviceToDevice,
                              inferenceStream);
        }

        m_Context->enqueueV3(inferenceStream);
        cudaError_t res = cudaStreamSynchronize(inferenceStream);
        if (res != cudaSuccess) {
            logger->log(Logger::LOG_LVL_ERROR, "CUDA stream synchronization failed: %s\r\n", cudaGetErrorString(res));
        }

        // Wrap m_OutputBuffer with exact stride — avoids pitch mismatch
        cv::cuda::GpuMat d_output(m_TensorHeight, m_TensorWidth, CV_32FC1,
                                  m_OutputBuffer, m_TensorWidth * sizeof(float));

        // Threshold to binary mask (0 or 255), then convert to uint8
        cv::cuda::GpuMat d_mask8u;
        cv::cuda::threshold(d_output, d_output, 0.5, 255.0, cv::THRESH_BINARY);
        d_output.convertTo(d_mask8u, CV_8UC1);
        // d_mask8u.download(outputFrame);

        // Convert single-channel mask to 3-channel BGR
        cv::cuda::GpuMat d_maskBgr;
        cv::cuda::cvtColor(d_mask8u, d_maskBgr, cv::COLOR_GRAY2BGR);

        // Zero out blue and red channels, keep green
        cv::cuda::GpuMat d_greenMask(d_maskBgr.size(), d_maskBgr.type(), cv::Scalar(0, 255, 0));
        cv::cuda::bitwise_and(d_maskBgr, d_greenMask, d_maskBgr);

        // Resize mask to match original frame size before overlay
        cv::cuda::GpuMat d_maskResized;
        cv::cuda::resize(d_maskBgr, d_maskResized, d_frame.size());

        // Overlay on original frame and download
        cv::cuda::addWeighted(d_frame, 0.7, d_maskResized, 0.3, 0, d_maskColor);
        d_maskColor.download(outputFrame);

        // Add line from center of lane to edge of detection. First find the midpoint and see if it falls
        // within the mask
        cv::Moments m = cv::moments(mask, true);
        if (m.m00 > 0) {
            // Centroid in mask coordinates
            cv::Point maskCenter(m.m10 / m.m00, m.m01 / m.m00);

            // Scale to outputFrame coordinates
            cv::Point drawCenter(
                maskCenter.x * outputFrame.cols / m_TensorWidth,
                maskCenter.y * outputFrame.rows / m_TensorHeight
            );

            cv::circle(outputFrame, drawCenter, 8, cv::Scalar(0, 0, 255), -1);

            if (mask.at<uchar>(maskCenter) == 255) {
                std::cout << "Car is within lane\r";
            }
        }

        return 0; // Return 0 on success
    }
};