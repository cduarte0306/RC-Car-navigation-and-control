#include "RcVisionControl.hpp"
#include "RcMessageLib.hpp"
#include "app/VideoStreamer.hpp"
#include "app/VideoFrame.hpp"
#include "utils/logger.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core/cuda.hpp>
#if __has_include(<opencv2/cudaimgcodecs.hpp>)
#include <opencv2/cudaimgcodecs.hpp>
#define RCVC_HAVE_CUDAIMGCODECS 1
#else
#define RCVC_HAVE_CUDAIMGCODECS 0
#endif

#include "Devices/StereoCam.hpp"

#include <opencv2/cudaimgproc.hpp>
#include <gst/gst.h>
#include <sstream>
#include <functional>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>

#define STREAM_PORT 5005

#define MODEL_PATH  "/opt/rc-car/models/model-small.onnx"


namespace Modules {
VisionControls::VisionControls(int moduleID, std::string name) :
    Base(moduleID, name), Adapter::CameraAdapter(name){
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Vision object initialized\r\n");

    setPeriod(1000);  // Set the timer thread to service ever second
}


/**
 * @brief Initialize the vision control module
 * 
 * @return int Error code
 */
int VisionControls::init(void) {
    using namespace cv::dnn;

    Logger* logger = Logger::getLoggerInst();
    // Initialize the network adapters
    m_TxAdapter = this->CommsAdapter->createNetworkAdapter(STREAM_PORT, "wlP1p1s0", Adapter::CommsAdapter::MaxUDPPacketSize);
    m_RxAdapter = this->CommsAdapter->createNetworkAdapter(STREAM_PORT, "enP8p1s0", Adapter::CommsAdapter::MaxUDPPacketSize);
    if (!m_TxAdapter || !m_RxAdapter) {
        logger->log(Logger::LOG_LVL_WARN, "Failed to create vision network adapters\r\n");
        return -1;
    }

    bool useCuda = false;

#ifdef HAVE_OPENCV_CUDAARITHM
    int cudaDevices = cv::cuda::getCudaEnabledDeviceCount();
    if (cudaDevices > 0) {
        logger->log(Logger::LOG_LVL_INFO, "OpenCV CUDA available; devices: %d\n", cudaDevices);
        cv::cuda::printShortCudaDeviceInfo(0);
        useCuda = true;
    } else {
        logger->log(Logger::LOG_LVL_WARN, "OpenCV CUDA runtime present but no enabled GPU detected\n");
    }
#else
    logger->log(Logger::LOG_LVL_WARN, "OpenCV built without CUDA support\n");
#endif
    // Check if the file exists
    if (std::filesystem::exists(MODEL_PATH)) {
        m_DnnNetDepth = cv::dnn::readNet(MODEL_PATH);
        if (m_DnnNetDepth.empty()) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to load DNN model at %s\n", MODEL_PATH);
        } else {
            if (useCuda) {
                m_DnnNetDepth.setPreferableBackend(DNN_BACKEND_CUDA);
                m_DnnNetDepth.setPreferableTarget(DNN_TARGET_CUDA);
                logger->log(Logger::LOG_LVL_INFO, "DNN model loaded with CUDA backend\n");
            } else {
                m_DnnNetDepth.setPreferableBackend(DNN_BACKEND_OPENCV);
                m_DnnNetDepth.setPreferableTarget(DNN_TARGET_CPU);
                logger->log(Logger::LOG_LVL_INFO, "DNN model loaded with CPU backend\n");
            }
        }
    } else {
        logger->log(Logger::LOG_LVL_WARN, "DNN model file not found at %s\n", MODEL_PATH);
        m_DnnNetDepth.setPreferableBackend(DNN_BACKEND_OPENCV);
        m_DnnNetDepth.setPreferableTarget(DNN_TARGET_CPU);
    }

    // Start receiving frames
    this->CommsAdapter->startReceive(
        *m_RxAdapter,
        std::bind(&VisionControls::recvFrame, this, std::placeholders::_1),
        false);
    return 0;
}


/**
 * @brief Configure the GStreamer pipeline
 * 
 * @param host Host IP address
 * @return int Error code
 */
int VisionControls::configurePipeline_(const std::string& host) {
    // Close if open
    return 0;
}


/**
 * @brief Received frame callback
 * 
 * @param pbuf Pointer to UDP receive buffer
 * @param length Length of data received
 */
void VisionControls::recvFrame(std::vector<char>& data) {
    if (data.empty()) return;
    Logger* logger = Logger::getLoggerInst();
    const std::size_t packetSize = data.size();
    if (packetSize < sizeof(Metadata)) {
        logger->log(Logger::LOG_LVL_WARN, "Frame fragment too small (%zu, need %zu)\n", packetSize, sizeof(Metadata));
        m_StreamInFrame.reset();
        return;
    }

    char* pbuf = reinterpret_cast<char*>(data.data());
    std::vector<uint8_t> segmentData;
    static uint64_t frameID = 0;

    // Decode the image
    uint8_t  numSegments  = 0;
    uint8_t  segmentID    = 0;
    uint32_t totalLength  = 0;
    uint16_t payloadLen   = 0;
    uint64_t seqId        = 0;
    int ret = VideoStreamer::decodePacket(pbuf, packetSize, const_cast<uint8_t&>(numSegments), const_cast<uint8_t&>(segmentID), const_cast<uint32_t&>(totalLength), const_cast<uint16_t&>(payloadLen), const_cast<uint64_t&>(seqId), segmentData);
    if (ret != 0) {
        logger->log(Logger::LOG_LVL_WARN, "Frame %llu decode packet error %d\n", static_cast<unsigned long long>(seqId), ret);
        m_StreamInFrame.reset();
        return;
    }

    // Basic header sanity
    if (numSegments == 0) {
        logger->log(Logger::LOG_LVL_WARN, "Frame %llu has zero segments\n", static_cast<unsigned long long>(seqId));
        m_StreamInFrame.reset();
        return;
    }

    if (segmentID >= numSegments) {
        logger->log(Logger::LOG_LVL_WARN, "Frame %llu invalid segment id %u / %u\n", static_cast<unsigned long long>(seqId), segmentID, numSegments);
        m_StreamInFrame.reset();
        return;
    }

    if (payloadLen == 0 || payloadLen > MaxPayloadSize) {
        logger->log(Logger::LOG_LVL_WARN, "Frame %llu invalid payload length %u (max %lld)\n", static_cast<unsigned long long>(seqId), payloadLen, static_cast<long long>(MaxPayloadSize));
        m_StreamInFrame.reset();
        return;
    }

    if (m_StreamInFrame.numSegments() > 0 && (m_StreamInFrame.frameID() != seqId)) {
        // Clear the map
        m_StreamInFrame.reset();
    }

    m_StreamInFrame.setFrameID(seqId);

    // Drop duplicates to avoid over-assembly
    if (m_StreamInFrame.getSegmentMap().count(segmentID) > 0) {
        logger->log(Logger::LOG_LVL_WARN,
                    "Frame %llu duplicate segment %u/%u\n",
                    static_cast<unsigned long long>(seqId),
                    segmentID,
                    numSegments);
        return;
    }

    m_StreamInFrame.append(static_cast<int>(segmentID), segmentData);

    // Store the frame ID
    frameID = seqId;

    // Once we have all segments, assemble the frame via VideoFrame helper
    if (m_StreamInFrame.numSegments() == numSegments) {
        const std::vector<uint8_t> assembled = m_StreamInFrame.bytes();

        if (assembled.size() != totalLength) {
            logger->log(Logger::LOG_LVL_WARN,
                        "Frame %llu size mismatch (expected %u, got %zu)\n",
                        static_cast<unsigned long long>(m_StreamInFrame.frameID()),
                        totalLength,
                        assembled.size());
            m_StreamInFrame.reset();
            return;  // drop corrupt frame
        }

        if (!assembled.empty()) {
            m_VideoRecorder.pushFrame(m_StreamInFrame);
        }
        
        m_StreamInFrame.reset();
    }
}


void VisionControls::decodeJPEG(cv::Mat& frame, const VideoFrame& frameEntry) {
    const auto& frameMap = frameEntry.getSegmentMap();
    const uint8_t numSegments = static_cast<uint8_t>(frameEntry.numSegments());

    if (frameMap.empty()) {
        frame.release();
        return;
    }

    // Validate all segments exist
    uint32_t totalSize = 0;
    for (uint8_t segID = 0; segID < numSegments; ++segID) {
        auto it = frameMap.find(segID);
        if (it == frameMap.end()) {
            frame.release();
            return;
        }
        totalSize += it->second.size();
    }

    // Reassemble
    std::vector<uint8_t> jpegFrame;
    jpegFrame.reserve(totalSize);

    for (uint8_t segID = 0; segID < numSegments; ++segID) {
        const auto& seg = frameMap.at(segID);
        jpegFrame.insert(jpegFrame.end(), seg.begin(), seg.end());
    }

    #if RCVC_HAVE_CUDAIMGCODECS
    cv::cuda::GpuMat gpu = cv::cuda::imdecode(jpegFrame, cv::IMREAD_COLOR);
    if (!gpu.empty()) {
        gpu.download(frame);
    } else {
        frame.release();
    }
    #else
    frame = cv::imdecode(jpegFrame, cv::IMREAD_COLOR);
    #endif

    if (frame.empty()) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_WARN, "Failed to decode JPEG (bytes=%zu, segments=%u)\n", jpegFrame.size(), numSegments);
    }
}


/**
 * @brief Process depth frame using DNN
 * 
 * @param frame Input/output frame
 */
void VisionControls::processDepth(cv::Mat& frame) {
    auto getOutputsNames = [](const cv::dnn::Net& net) {
        std::vector<std::string> names;
        std::vector<int32_t> out_layers = net.getUnconnectedOutLayers();
        std::vector<std::string> layers_names = net.getLayerNames();
        names.resize(out_layers.size());
        for (size_t i = 0; i < out_layers.size(); ++i) {
            names[i] = layers_names[out_layers[i] - 1];
        }
        return names;
    };

    static bool logMissingModel = false;
    if (m_DnnNetDepth.empty()) {
        if (!logMissingModel) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "CamModeDepth selected but DNN model is not loaded\n");
            logMissingModel = true;
        }
        return;
    }

    if (frame.empty()) {
        return;
    }

    // Pick input size based on model name hint: small => 256, otherwise 384
    constexpr int kDepthInputLarge = 384;
    constexpr int kDepthInputSmall = 256;
    const bool isSmall = std::string(MODEL_PATH).find("small") != std::string::npos;
    const int kDepthInput = isSmall ? kDepthInputSmall : kDepthInputLarge;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(kDepthInput, kDepthInput));

    // MiDaS-style preprocessing: scale 1/255, RGB order, mean subtraction
    const cv::Scalar mean(123.675, 116.28, 103.53);
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0f / 255.0f, cv::Size(kDepthInput, kDepthInput), mean, true, false);

    m_DnnNetDepth.setInput(blob);

    cv::Mat output;
    try {
        output = m_DnnNetDepth.forward(getOutputsNames(m_DnnNetDepth)[0]);
    } catch (const cv::Exception& e) {
        Logger::getLoggerInst()->log(
            Logger::LOG_LVL_ERROR,
            "DNN forward failed: %s (blob shape: NCHW=%d,%d,%d,%d)\n",
            e.what(),
            blob.size[0], blob.size[1], blob.size[2], blob.size[3]);
        return;
    }

    if (output.empty()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "DNN forward returned empty output\n");
        return;
    }

    cv::Mat depth;
    if (output.dims == 4 && output.size[1] == 1) {
        const int outH = output.size[2];
        const int outW = output.size[3];
        depth = cv::Mat(outH, outW, CV_32F, output.ptr<float>());
    } else if (output.dims == 3) {
        const int outH = output.size[1];
        const int outW = output.size[2];
        const std::vector<int32_t> sz = { outH, outW };
        depth = cv::Mat(static_cast<int32_t>(sz.size()), sz.data(), CV_32F, output.ptr<float>());
    } else {
        static bool loggedUnexpected = false;
        if (!loggedUnexpected) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Unexpected DNN output dims=%d\n", output.dims);
            loggedUnexpected = true;
        }
        return;
    }

    cv::Mat depthResized;
    cv::resize(depth, depthResized, frame.size());

    double minVal = 0.0, maxVal = 0.0;
    cv::minMaxLoc(depthResized, &minVal, &maxVal);
    const double range = maxVal - minVal;

    cv::Mat depthNorm;
    if (std::abs(range) < 1e-6) {
        depthNorm = cv::Mat::zeros(depthResized.size(), CV_8U);
    } else {
        depthResized.convertTo(depthNorm, CV_8U, 255.0 / range, -minVal * 255.0 / range);
    }

    cv::Mat depthColor;
    cv::applyColorMap(depthNorm, depthColor, cv::COLORMAP_MAGMA);

    cv::addWeighted(depthColor, 0.6, frame, 0.4, 0.0, frame);
}


/**
 * @brief Process stereo and compiles a stereo frame pair
 * 
 * @param stereoFrame Input/output stereo frame pair
 */
void VisionControls::processStereo(cv::Mat& stereoFrame, std::pair<cv::Mat, cv::Mat>& stereoFramePair) {
    using namespace std;
    using namespace cv;
    using namespace dnn;

    cv::Mat& frameL = stereoFramePair.first;
    cv::Mat& frameR = stereoFramePair.second;

    // For testing purposes. We just superimpose the two frames into one
    cv::addWeighted(frameL, 0.5, frameR, 0.5, 0.0, stereoFrame);
}


/**
 * @brief Process a frame (placeholder for future processing)
 * 
 * @param frame Input/output frame
 */
void VisionControls::processFrame(cv::Mat& frame) {
    using namespace std;
    using namespace cv;
    using namespace dnn;

    switch (m_CamSettings.mode) {
        case CamModeNormal:
            /* code */
            break;
        
        case CamModeDepth: {
            if (frame.channels() == 4) {
                cv::Mat bgr;
                cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
                frame = bgr;
            }
            processDepth(frame);
            break;
        }

        default:
            break;
    }
}


/** * @brief Process a stereo frame pair (placeholder for future processing)
 * 
 * @param frame Input/output frame pair
 */
void VisionControls::processFrame(std::pair<cv::Mat, cv::Mat>& stereoFrame) {
    using namespace std;
    using namespace cv;
    using namespace dnn;

    cv::Mat& frameL = stereoFrame.first;
    cv::Mat& frameR = stereoFrame.second;

    switch (m_CamSettings.mode) {
        case CamModeNormal:
            /* code */
            break;
        
        case CamModeDepth: {
            if (frameL.channels() == 4) {
                cv::Mat bgr;
                cv::cvtColor(frameL, bgr, cv::COLOR_BGRA2BGR);
                frameL = bgr;
            }
            processDepth(frameL);
            break;
        }

        default:
            break;
    }
}


/**
 * @brief Module command handler
 * 
 * @param pbuf Pointer to command buffer
 * @param len Length of command buffer
 * @return int Error code
 */
int VisionControls::moduleCommand_(std::vector<char>& buffer) {
    // Currently no commands implemented
    CameraCommand* cmd = reinterpret_cast<CameraCommand*>(buffer.data());
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "VisionControls received command: %d\n", cmd->command);

    switch (cmd->command) 
    {
        case CmdSetFrameRate:
            /* code */
            break;

        case CmdStartStream:
            /* code */
            break;

        case CmdStopStream:
            /* code */
            break;

        case CmdStreamMode: {
            if (cmd->data.u8 >= StreamModeMax) {
                logger->log(Logger::LOG_LVL_WARN, "Invalid stream mode %d\n", cmd->data.u8);
                return -1;
            }

            const char* modes[] = {"Camera", "Simulation", "CameraMono"};
            logger->log(Logger::LOG_LVL_INFO, "Configuring stream mode to %s, cmd: %d\n", modes[cmd->data.u8], cmd->data.u8);
            m_StreamStats.streamInStatus = cmd->data.u8;
            if (cmd->data.u8 != StreamSim) {
                m_VideoRecorder.resetPlayback();
            }
            break;
        }
        case CmdSelCameraMode:
            logger->log(Logger::LOG_LVL_INFO, "Setting camera mode to %d\n", cmd->data.u8);
            m_CamSettings.mode = cmd->data.u8;
            break;

        case CmdClrVideoRec:
            logger->log(Logger::LOG_LVL_INFO, "Clearing video recording buffer\n");
            m_StreamInFrame.reset();
            m_VideoRecorder.clear();
            break;

        default:
            break;
    }
    return 0;
}


/**
 * @brief Thread timer handler
 * 
 */
void VisionControls::OnTimer(void) {
}


void VisionControls::mainProc() {
    Logger* logger = Logger::getLoggerInst();

    // Handles jitter smoothing and transmission in its own thread
    VideoStreamer streamer(
        *m_TxAdapter,
        [this]() {
            std::lock_guard<std::mutex> lock(m_HostIPMutex);
            return m_HostIP;
        },
        35,
        1);
    streamer.start();
    m_VideoRecorder.setFrameRate(VideoRecording::FrameRate::_30Fps);

    Devices::StereoCam cam(0, 1);
    cam.start(1280, 720, 30);
    cv::Mat frameL;
    cv::Mat frameR;
    cv::Mat frameStereo;

    cv::Mat frameSim;
    cv::Ptr<cv::StereoBM> stereoBM = cv::StereoBM::create();
    std::pair<cv::Mat, cv::Mat> stereoFramePair;
    int16_t xGyro, yGyro, zGyro;
    while (m_Running.load()) {
        switch(m_StreamStats.streamInStatus) {
            case StreamCameraMono:
                cam.read(frameL, frameR, xGyro, yGyro, zGyro);
                stereoFramePair = std::make_pair(frameL, frameR);
                processStereo(frameStereo, stereoFramePair);
                streamer.pushFrame(frameStereo);
                break;

            case StreamCamera:
                cam.read(frameL, frameR, xGyro, yGyro, zGyro);
                stereoFramePair = std::make_pair(frameL, frameR);
                streamer.pushFrame(stereoFramePair);
                break;

            case StreamSim: {
                // We draw a frame from the recording object
                VideoFrame simFrame = m_VideoRecorder.getNextFrame();
                if (simFrame.numSegments() == 0) {
                    // No frames available, wait a bit
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                decodeJPEG(frameSim, simFrame);
                if (frameSim.empty()) {
                    logger->log(Logger::LOG_LVL_WARN, "Could not decode JPEG from simulation frame\n");
                    continue;
                }
                processFrame(frameSim);
                streamer.pushFrame(frameSim);
                break;
            }

            default:
                break;
        }
    }
}
}
