#include "TensorRTEngine.hpp"
#include "utils/logger.hpp"
#include <openssl/md5.h>

#include <fstream>
#include <filesystem>
#include <vector>
#include <cuda_runtime_api.h>
#include <openssl/md5.h>



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
    bool recompileEngineFile = false;

    // Check if the directory for the destination engine file exists, if not create it
    std::filesystem::path engineFilePath(enginePath);
    if (!std::filesystem::exists(engineFilePath.parent_path())) {
        std::filesystem::create_directories(engineFilePath.parent_path());
    } else if (std::filesystem::exists(enginePath)) {
        // Check if the onnx file checksum matches the store checksum
        std::string istrm;
        std::string calcChecksum;
        std::ifstream file((std::filesystem::path(enginePath).parent_path()/std::filesystem::path(enginePath).stem()).generic_string() + std::string(".md5"));
        if (file.is_open()) {
            file >> istrm;
            std::string onnxFile(onnxModelPath);
            calcChecksum = calculateMd5checksum(onnxFile);
            if (istrm != calcChecksum) {
                logger->log(Logger::LOG_LVL_INFO, "New checksum on ONNX model detected. Re-compiling engine file...\r\n");
                std::filesystem::remove(enginePath);  // We remove the engine file to force a to re-compile
                recompileEngineFile = true;
            }
        } else {
            recompileEngineFile = true;
        }
    }

    if (std::filesystem::exists(enginePath) && !recompileEngineFile) {
        logger->log(Logger::LOG_LVL_INFO, "Engine file already exists...\r\n");
        return 0;
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

    std::string modelPath(onnxModelPath);
    std::string md5sum = calculateMd5checksum(modelPath);

    // Write the checksum to the models/ directory
    std::ofstream md5File((std::filesystem::path(enginePath).parent_path()/std::filesystem::path(enginePath).stem()).generic_string() + std::string(".md5"));
    if (md5File.is_open()) {
        md5File << std::hex << md5sum;
        md5File.close();   
    }

    return 0;
}


std::string TensorRTEngine::calculateMd5checksum(std::string& fileName) {
    // Check checksum file on ONNX to make sure no changes where made. If different, then we must rebuild the engine.
    std::filesystem::path onnxPath = fileName;
    std::filesystem::path path = onnxPath.parent_path();
    std::filesystem::path name = path.stem();
    path = path / name;
    
    std::ofstream engineChecksum;
    std::stringstream ss;
    
    unsigned char result[MD5_DIGEST_LENGTH];
    MD5_CTX mdContext;
    MD5_Init(&mdContext);        

    std::ifstream file(fileName, std::ios::binary);
    if (!file) return "";

    char buffer[4096];
    while (file.read(buffer, sizeof(buffer)) || file.gcount() > 0) {
        MD5_Update(&mdContext, buffer, file.gcount());
    }

    MD5_Final(result, &mdContext);
    
    for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)result[i];

    return ss.str();
}