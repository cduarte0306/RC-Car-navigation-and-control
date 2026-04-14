#include "TensorRTEngine.hpp"
#include "utils/logger.hpp"

#include <fstream>
#include <filesystem>
#include <vector>
#include <cuda_runtime_api.h>


void TrtLogger::log(Severity severity, const char* msg) noexcept {
    if (severity <= Severity::kWARNING) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "[TensorRT][%s] %s\r\n", m_Name.c_str(), msg);
    }
}

static TrtLogger sBuildLogger("EngineBuilder");
static cudaStream_t inferenceStream;


TensorRTEngine::TensorRTEngine() {}

TensorRTEngine::TensorRTEngine(const char* modelDir) : m_OnnxPath(modelDir) {
    m_Name = std::filesystem::path(modelDir).stem().string();
    m_TrtLogger.setName(m_Name);
}

TensorRTEngine::TensorRTEngine(const std::string& name) : m_Name(name) {
    m_TrtLogger.setName(m_Name);
}

TensorRTEngine::TensorRTEngine(const char* onnxPath, const char* enginePath)
    : m_OnnxPath(onnxPath), m_EnginePath(enginePath) {
    m_Name = std::filesystem::path(enginePath).stem().string();
    m_TrtLogger.setName(m_Name);
}

TensorRTEngine::~TensorRTEngine() {
    cudaFree(m_InputBuffer);
    cudaFree(m_OutputBuffer);
    cudaStreamDestroy(inferenceStream);
    delete m_Context;
    delete m_Engine;
    delete m_Runtime;
}


int TensorRTEngine::init(void) {
    Logger* logger = Logger::getLoggerInst();

    m_Runtime = nvinfer1::createInferRuntime(m_TrtLogger);
    std::filesystem::path enginePath = m_EnginePath
        ? std::filesystem::path(m_EnginePath)
        : std::filesystem::path(std::string(m_OnnxPath)).replace_extension(".engine");

    std::ifstream engineFile(enginePath, std::ios::binary);
    if (!engineFile) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to open engine file: %s\r\n", enginePath.c_str());
        return -1;
    }

    std::vector<char> modelData((std::istreambuf_iterator<char>(engineFile)), std::istreambuf_iterator<char>());
    engineFile.close();

    m_Engine = m_Runtime->deserializeCudaEngine(modelData.data(), modelData.size());
    if (!m_Engine) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to deserialize engine: %s\r\n", enginePath);
        return -1;
    }

    m_Context = m_Engine->createExecutionContext();
    if (!m_Context) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create execution context\r\n");
        return -1;
    }
    return 0;
}

int TensorRTEngine::createEngineFile(const char* onnxModelPath, const char* enginePath, bool enableFP16) {
    using namespace nvonnxparser;
    Logger* logger = Logger::getLoggerInst();

    if (std::filesystem::exists(enginePath)) {
        logger->log(Logger::LOG_LVL_INFO, "Engine file already exists at %s. Skipping engine creation.\r\n", enginePath);
        return 0;
    }

    // Check if the directory for the destination engine file exists, if not create it
    std::filesystem::path engineFilePath(enginePath);
    if (!std::filesystem::exists(engineFilePath.parent_path())) {
        std::filesystem::create_directories(engineFilePath.parent_path());
    }

    logger->log(Logger::LOG_LVL_INFO, "No engine file found at %s. Generating...\r\n", enginePath);
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(sBuildLogger);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(1 << static_cast<int>(
        nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
    IParser* parser = nvonnxparser::createParser(*network, sBuildLogger);
    parser->parseFromFile(onnxModelPath, static_cast<int>(nvinfer1::ILogger::Severity::kWARNING));

    bool failure = false;
    for (int i = 0; i < parser->getNbErrors(); ++i) {
        logger->log(Logger::LOG_LVL_ERROR, "ONNX Parser error: %s\r\n", parser->getError(i)->desc());
        failure = true;
    }

    if (failure) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to parse ONNX model. Aborting engine creation.\r\n");
        delete parser; delete network; delete builder;
        return -1;
    }

    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1U << 30);
    if (enableFP16) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }

    // Load or create a timing cache to speed up repeated engine builds
    std::filesystem::path cachePath =
        std::filesystem::path(enginePath).replace_extension(".cache");
    nvinfer1::ITimingCache* timingCache = nullptr;
    {
        std::ifstream cacheFile(cachePath, std::ios::binary);
        if (cacheFile) {
            std::vector<char> cacheData((std::istreambuf_iterator<char>(cacheFile)),
                                         std::istreambuf_iterator<char>());
            timingCache = config->createTimingCache(cacheData.data(), cacheData.size());
            logger->log(Logger::LOG_LVL_INFO, "Loaded timing cache from %s\r\n", cachePath.c_str());
        } else {
            timingCache = config->createTimingCache(nullptr, 0);
        }
        config->setTimingCache(*timingCache, false);
    }

    nvinfer1::IHostMemory* serializedModel = builder->buildSerializedNetwork(*network, *config);

    // Save timing cache for next build
    if (timingCache) {
        nvinfer1::IHostMemory* serializedCache = timingCache->serialize();
        if (serializedCache) {
            std::ofstream cacheOut(cachePath, std::ios::binary);
            cacheOut.write(reinterpret_cast<const char*>(serializedCache->data()),
                           serializedCache->size());
            delete serializedCache;
            logger->log(Logger::LOG_LVL_INFO, "Saved timing cache to %s\r\n", cachePath.c_str());
        }
        delete timingCache;
    }

    if (!serializedModel) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to build TensorRT engine.\r\n");
        delete parser; delete network; delete config; delete builder;
        return -1;
    }

    std::ofstream outFile(enginePath, std::ios::binary);
    if (!outFile) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to open engine file for writing: %s\r\n", enginePath);
        delete parser; delete network; delete config; delete builder;
        return -1;
    }
    outFile.write(reinterpret_cast<const char*>(serializedModel->data()), serializedModel->size());
    outFile.close();
    delete serializedModel;
    logger->log(Logger::LOG_LVL_INFO, "Engine saved to %s\r\n", enginePath);

    delete parser;
    delete network;
    delete config;
    delete builder;

    return 0;
}
