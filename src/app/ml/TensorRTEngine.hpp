#pragma once

#include "NvInfer.h"      // core TensorRT: builder, network, engine
#include "NvOnnxParser.h" // ONNX parser: reads your .onnx file

#include <string>


class TrtLogger : public nvinfer1::ILogger {
public:
    TrtLogger() = default;
    explicit TrtLogger(const std::string& name) : nvinfer1::ILogger(), m_Name(name) {}
    void log(Severity severity, const char* msg) noexcept override;
    void setName(const std::string& name) { m_Name = name; }
    const std::string& getName() const { return m_Name; }

private:
    std::string m_Name = "Unnamed";
};


class TensorRTEngine {
public:
    TensorRTEngine();
    TensorRTEngine(const char* modelDir);
    TensorRTEngine(const std::string& name);
    // Explicit engine path — skips deriving path from ONNX path
    TensorRTEngine(const char* onnxPath, const char* enginePath);

    ~TensorRTEngine();

    /**
     * @brief Initializes context object
     * 
     * @return int 
     *  0 on success
     * -1 on failure
     */
    int init();

    /**
     * @brief Get the Context object
     * 
     * @return nvinfer1::IExecutionContext* Pointer to the execution context, which is used to execute inference on the TensorRT engine. The context holds the state of the engine and allows you to run inference by providing input data and retrieving output results. 
     */
    nvinfer1::IExecutionContext* getContext() const { return m_Context; }

    /**
     * @brief Get the Engine object
     * 
     * @return nvinfer1::ICudaEngine* Pointer to the CUDA engine, which is the compiled version of the neural network model optimized for inference on NVIDIA GPUs. The engine contains the optimized layers and operations that can be executed efficiently during inference.
     */
    nvinfer1::ICudaEngine* getEngine() const { return m_Engine; }

    /**
     * @brief Get the Runtime object
     * 
     * @return nvinfer1::IRuntime* Pointer to the runtime, which is responsible for managing the execution of the TensorRT engine. The runtime provides functions to create and manage execution contexts, as well as to handle memory allocation and deallocation for input and output buffers during inference.
     */
    nvinfer1::IRuntime* getRuntime() const { return m_Runtime; }

    /**
     * @brief Converts an ONNX model to a TensorRT engine. This function will parse the ONNX model and build a TensorRT engine that can be used for inference.
     * 
     * @param onnxModelPath Path to the ONNX model file.
     * @return int 
     */
    static int createEngineFile(const char* onnxModelPath, const char* enginePath = "lanenet.engine", bool enableFP16 = true);

    /**
     * @brief Set the name of the model (for logging purposes)
     * 
     * @param name Name of the model
     */
    void setName(const char* name) { m_Name = name; }

protected:

    /**
     * @brief Calculates MD5 checksum of .onnx file
     * 
     * @param fileName ONNX file path
     * @return std::string Resultant checksum (Hex)
     */
    static std::string calculateMd5checksum(std::string& fileName);

    const char* m_OnnxPath   = nullptr;
    const char* m_EnginePath = nullptr;  // explicit override; if null, derived from m_OnnxPath

    // Engine object 
    nvinfer1::IRuntime* m_Runtime = nullptr;
    nvinfer1::ICudaEngine* m_Engine = nullptr;
    nvinfer1::IExecutionContext* m_Context = nullptr;

    uint32_t m_TensorWidth = 0;
    uint32_t m_TensorHeight = 0;

    static constexpr const char* INPUT_TENSOR_NAME = "image";
    static constexpr const char* OUTPUT_TENSOR_NAME = "logits";

    /**
     * @brief Input buffer for the engine. This will hold the preprocessed input image data that will be fed into the engine during inference. The buffer will be allocated based on the input tensor dimensions and data type defined in the engine.
     * 
     */
    void* m_InputBuffer = nullptr;

    /**
     * @brief Output buffer for the engine. This will hold the inference results produced by the engine after processing the input data. The buffer will be allocated based on the output tensor dimensions and data type defined in the engine. During inference, the engine will write the output data to this buffer, which can then be post-processed to extract lane markings or other relevant information.
     * 
     */
    void* m_OutputBuffer = nullptr;

    /**
     * @brief Name of model
     *
     */
    std::string m_Name;

    /**
     * @brief Per-instance TensorRT logger tagged with the engine name
     */
    TrtLogger m_TrtLogger;

};

#pragma endregion