#include "RcVisionControl.hpp"
#include "RcMessageLib.hpp"
#include "app/video/VideoStreamer.hpp"
#include "app/video/VideoFrame.hpp"
#include "utils/logger.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core/cuda.hpp>

#include <vpi/OpenCVInterop.hpp>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>

#if __has_include(<opencv2/cudaimgcodecs.hpp>)
#include <opencv2/cudaimgcodecs.hpp>
#define RCVC_HAVE_CUDAIMGCODECS 1
#else
#define RCVC_HAVE_CUDAIMGCODECS 0
#endif
#if __has_include(<opencv2/cudastereo.hpp>)
#include <opencv2/cudastereo.hpp>
#define RCVC_HAVE_CUDASTEREO 1
#else
#define RCVC_HAVE_CUDASTEREO 0
#endif

#include "Devices/StereoCam.hpp"
#include <nlohmann/json.hpp>

#include <sstream>
#include <opencv2/cudaimgproc.hpp>
#include <sstream>
#include <functional>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

#include "cudaVision.hpp"


#define STREAM_PORT 5005

#define MODEL_PATH  "/opt/rc-car/models/model-small.onnx"

#define CHECK_STATUS(STMT)                                                                  \
    do                                                                                      \
    {                                                                                       \
        VPIStatus status = (STMT);                                                          \
        if (status != VPI_SUCCESS)                                                          \
        {                                                                                   \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];                                     \
            vpiGetLastStatusMessage(buffer, sizeof(buffer));                                \
            std::ostringstream ss;                                                          \
            ss << "line " << __LINE__ << " " << vpiStatusGetName(status) << ": " << buffer; \
            throw std::runtime_error(ss.str());                                             \
        }                                                                                   \
    } while (0);

    
const char* StorageLocation = "/data/calibration-data/streaming-profile.json";

namespace {
static bool isValidSgmDisparities(int value) {
    return value == 64 || value == 128 || value == 256;
}

static constexpr int kHardMaxDisparity = 64;
static constexpr int kHardWindowSize = 10;
static constexpr int kPayloadMaxDisparity = 255;

static int clampInt(int value, int lo, int hi) {
    return std::max(lo, std::min(value, hi));
}

}

namespace Modules {
void VisionControls::sanitizeVpiStereoSettings(VisionControls::CameraSettings &s)
{
    // Hard-coded for now (must match payload creation params).
    s.numDisparities = kHardMaxDisparity;

    // VPI CUDA stereo requires minDisparity in [0, maxDisparity].
    s.minDisparity = clampInt(s.minDisparity, 0, s.numDisparities);

    // VPI CUDA penalty constraints: P1 > 0, P2 >= P1, P2 < 256.
    // If profile contains legacy OpenCV values (thousands), reset to VPI-friendly defaults.
    if (s.p1 <= 0 || s.p1 > 255) s.p1 = 3;
    if (s.p2 <= 0 || s.p2 > 255) s.p2 = 48;
    s.p1 = clampInt(s.p1, 1, 255);
    s.p2 = clampInt(s.p2, s.p1, 255);

    // Stored uniquenessRatio is historically [0..30] in this codebase.
    s.uniquenessRatio = clampInt(s.uniquenessRatio, 0, 30);

    // Runtime VPI stereo params (except windowSize).
    if (s.maxDisparity != 0 && s.maxDisparity != kPayloadMaxDisparity) {
        s.maxDisparity = 0;
    }
    s.confidenceThreshold = clampInt(s.confidenceThreshold, 0, 65535);
    s.vpiQuality = clampInt(s.vpiQuality, 1, 8);
    s.confidenceType = clampInt(s.confidenceType,
                                static_cast<int>(VPI_STEREO_CONFIDENCE_ABSOLUTE),
                                static_cast<int>(VPI_STEREO_CONFIDENCE_INFERENCE));
    if (!(s.p2Alpha == 0 || s.p2Alpha == 1 || s.p2Alpha == 2 || s.p2Alpha == 4 || s.p2Alpha == 8)) {
        s.p2Alpha = 0;
    }
    if (s.uniqueness < 0.0f) {
        s.uniqueness = -1.0f;
    } else if (s.uniqueness > 1.0f) {
        s.uniqueness = 1.0f;
    }
    s.numPasses = clampInt(s.numPasses, 1, 3);
}

VisionControls::~VisionControls()
{
    // Best-effort cleanup; VPI destroy functions are NULL-safe.
    if (m_ImgL)          { vpiImageDestroy(m_ImgL);          m_ImgL          = nullptr; }
    if (m_ImgR)          { vpiImageDestroy(m_ImgR);          m_ImgR          = nullptr; }
    if (m_ImgL_8u)       { vpiImageDestroy(m_ImgL_8u);       m_ImgL_8u       = nullptr; }
    if (m_ImgR_8u)       { vpiImageDestroy(m_ImgR_8u);       m_ImgR_8u       = nullptr; }
    if (m_ImgL_270p)     { vpiImageDestroy(m_ImgL_270p);     m_ImgL_270p     = nullptr; }
    if (m_ImgR_270p)     { vpiImageDestroy(m_ImgR_270p);     m_ImgR_270p     = nullptr; }
    if (m_Disparity)     { vpiImageDestroy(m_Disparity);     m_Disparity     = nullptr; }
    if (m_ConfidenceMap) { vpiImageDestroy(m_ConfidenceMap); m_ConfidenceMap = nullptr; }
    if (m_Stereo)        { vpiPayloadDestroy(m_Stereo);      m_Stereo        = nullptr; }
    if (m_VpiStream)     { vpiStreamDestroy(m_VpiStream);    m_VpiStream     = nullptr; }
}

VisionControls::VisionControls(int moduleID, std::string name) :
    Base(moduleID, name), Adapter::CameraAdapter(name) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Vision object initialized\r\n");

    setPeriod(1000);  // Set the timer thread to service ever second
    
    VisionControls::loadStreamingProfile(m_CamSettings);
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
    m_TxAdapter = this->CommsAdapter->createNetworkAdapter(getName(), 0, STREAM_PORT, "wlP1p1s0", Adapter::CommsAdapter::MaxUDPPacketSize);
    m_EthAdapter = this->CommsAdapter->createNetworkAdapter(getName(), 0, STREAM_PORT, "enP8p1s0", Adapter::CommsAdapter::MaxUDPPacketSize);

    m_TxAdapter->setParent(this->getName());
    m_EthAdapter->setParent(this->getName());

    if (!m_TxAdapter || !m_EthAdapter) {
        logger->log(Logger::LOG_LVL_WARN, "Failed to create vision network adapters\r\n");
        return -1;
    }

    // Create the streamer only once the TX adapter exists; otherwise we'd bind a dangling reference.
    m_VideoStreamer = std::make_unique<Vision::VideoStreamer>(*m_TxAdapter, *m_EthAdapter, 100);

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
        *m_EthAdapter,
        std::bind(&VisionControls::onEthRecv, this, std::placeholders::_1),
        false);

    // Register telemetry port
    int ret = this->TlmAdapter->registerTelemetrySource(this->getName());
    if (ret < 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to register vision control as telemetry source\r\n");
        return -1;
    }

    // Create camera object
    m_Cam = std::make_unique<Devices::StereoCam>(0, 1);
    m_Cam->start(CAM_WIDTH, CAM_HEIGHT, 30);

    m_VideoStreamer->setJpegQuality(m_CamSettings.quality);  // Default to maximum quality
    m_VideoStreamer->setStreamFrameRate(static_cast<Vision::VideoStreamer::FrameRate>(m_CamSettings.frameRate));
    
    m_VideoRecorder.setFrameRate(Vision::VideoRecording::FrameRate::_30Fps);

    // Create VPI stream
    CHECK_STATUS(vpiStreamCreate(0, &m_VpiStream));
    backend_ = VPI_BACKEND_CUDA;

    // Initialise format-conversion, stereo runtime, and stereo creation params
    CHECK_STATUS(vpiInitConvertImageFormatParams(&m_ConvParams));
    CHECK_STATUS(vpiInitStereoDisparityEstimatorParams(&m_StereoParams));
    CHECK_STATUS(vpiInitStereoDisparityEstimatorCreationParams(&m_CreateParams));

    VisionControls::sanitizeVpiStereoSettings(m_CamSettings);

    // Set creation parameters
    m_CreateParams.maxDisparity     = 255;
    m_CreateParams.downscaleFactor  = 1;
    m_CreateParams.includeDiagonals = 1;

    // Allocate input image buffers in Y16_ER format
    const VPIImageFormat inputFormat = VPI_IMAGE_FORMAT_Y16_ER;
    CHECK_STATUS(vpiImageCreate(CAM_WIDTH, CAM_HEIGHT, inputFormat, 0, &m_ImgL_8u));
    CHECK_STATUS(vpiImageCreate(CAM_WIDTH, CAM_HEIGHT, inputFormat, 0, &m_ImgR_8u));
    CHECK_STATUS(vpiImageCreate(w_, h_, inputFormat, 0, &m_ImgL_270p));
    CHECK_STATUS(vpiImageCreate(w_, h_, inputFormat, 0, &m_ImgR_270p));

    // Create stereo disparity estimator + output buffers
    CHECK_STATUS(vpiCreateStereoDisparityEstimator(backend_, w_, h_, inputFormat, &m_CreateParams, &m_Stereo));
    CHECK_STATUS(vpiImageCreate(w_, h_, VPI_IMAGE_FORMAT_S16, 0, &m_Disparity));
    CHECK_STATUS(vpiImageCreate(w_, h_, VPI_IMAGE_FORMAT_U16, 0, &m_ConfidenceMap));

    logger->log(Logger::LOG_LVL_INFO, "Vision control module initialized\r\n");
    return 0;
}


int VisionControls::saveStreamingProfile(VisionControls::CameraSettings& settings) {
    const std::filesystem::path profilePath(StorageLocation);
    const std::filesystem::path parentDir = profilePath.has_parent_path() ? profilePath.parent_path() : std::filesystem::path{};

    // If a previous version accidentally created a directory at the file path, move it aside.
    if (std::filesystem::exists(profilePath) && std::filesystem::is_directory(profilePath)) {
        std::error_code ec;
        std::filesystem::path bak = profilePath;
        bak += ".bakdir";
        std::filesystem::rename(profilePath, bak, ec);
        if (ec) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                         "Streaming profile path is a directory and could not be moved: %s\n",
                                         StorageLocation);
            return -1;
        }
    }

    if (!parentDir.empty() && !std::filesystem::exists(parentDir)) {
        std::error_code ec;
        std::filesystem::create_directories(parentDir, ec);
        if (ec) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                         "Failed to create streaming profile directory: %s\n",
                                         parentDir.string().c_str());
            return -1;
        }
    }

    nlohmann::json profileSettings;
    profileSettings["quality"     ] = settings.quality;
    profileSettings["fps"         ] = settings.frameRate;
    profileSettings["disparities" ] = settings.numDisparities;
    profileSettings["numBlocks"   ] = settings.numBlocks;
    profileSettings["preFilterType"    ] = settings.preFilterType;
    profileSettings["preFilterSize"    ] = settings.preFilterSize;
    profileSettings["preFilterCap"     ] = settings.preFilterCap;
    profileSettings["textureThreshold" ] = settings.textureThreshold;
    profileSettings["uniquenessRatio"  ] = settings.uniquenessRatio;
    profileSettings["speckleWindowSize"] = settings.speckleWindowSize;
    profileSettings["speckleRange"     ] = settings.speckleRange;
    profileSettings["disp12MaxDiff"    ] = settings.disp12MaxDiff;
    profileSettings["minDisparity"     ] = settings.minDisparity;
    profileSettings["p1"               ] = settings.p1;
    profileSettings["p2"               ] = settings.p2;
    profileSettings["sgmMode"          ] = settings.sgmMode;
    profileSettings["maxDisparity"       ] = settings.maxDisparity;
    profileSettings["confidenceThreshold"] = settings.confidenceThreshold;
    profileSettings["confidenceType"     ] = settings.confidenceType;
    profileSettings["vpiQuality"         ] = settings.vpiQuality;
    profileSettings["p2Alpha"            ] = settings.p2Alpha;
    profileSettings["uniqueness"         ] = settings.uniqueness;
    profileSettings["numPasses"          ] = settings.numPasses;
    profileSettings["zMax"               ] = settings.zMax;
    profileSettings["zMin"               ] = settings.zMin;

    std::ofstream out(profilePath, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Failed to write streaming profile: %s\n",
                                     StorageLocation);
        return -1;
    }
    out << profileSettings.dump(2);
    out.flush();
    return 0;
}


int VisionControls::loadStreamingProfile(VisionControls::CameraSettings& settings) {
    const std::filesystem::path profilePath(StorageLocation);

    if (!std::filesystem::exists(profilePath)) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN,
                                     "Streaming profile does not exist: %s\n",
                                     StorageLocation);
        return 0;
    }

    // If a previous version accidentally created a directory at the file path, move it aside.
    if (std::filesystem::is_directory(profilePath)) {
        std::error_code ec;
        std::filesystem::path bak = profilePath;
        bak += ".bakdir";
        std::filesystem::rename(profilePath, bak, ec);
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN,
                                     "Streaming profile path is a directory; moved aside to %s\n",
                                     bak.string().c_str());
        return 0;
    }

    // Read from the file
    std::ifstream profileFile(profilePath);
    if (!profileFile.is_open()) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Failed to open streaming profile: %s\n",
                                     StorageLocation);
        return -1;
    }

    nlohmann::json profileSettings;

    try {
        profileFile >> profileSettings;
    } catch (const std::exception& e) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                     "Failed to parse streaming profile container %s: %s\n",
                                     StorageLocation,
                                     e.what());
        return -1;
    }

    if (profileSettings.contains("quality")) settings.quality = profileSettings["quality"].get<int>();
    if (profileSettings.contains("fps")) settings.frameRate = profileSettings["fps"].get<int>();
    if (profileSettings.contains("disparities")) settings.numDisparities = profileSettings["disparities"].get<int>();
    if (profileSettings.contains("numBlocks")) settings.numBlocks = profileSettings["numBlocks"].get<int>();
    if (profileSettings.contains("preFilterType")) settings.preFilterType = profileSettings["preFilterType"].get<int>();
    if (profileSettings.contains("preFilterSize")) settings.preFilterSize = profileSettings["preFilterSize"].get<int>();
    if (profileSettings.contains("preFilterCap")) settings.preFilterCap = profileSettings["preFilterCap"].get<int>();
    if (profileSettings.contains("textureThreshold")) settings.textureThreshold = profileSettings["textureThreshold"].get<int>();
    if (profileSettings.contains("uniquenessRatio")) settings.uniquenessRatio = profileSettings["uniquenessRatio"].get<int>();
    if (profileSettings.contains("speckleWindowSize")) settings.speckleWindowSize = profileSettings["speckleWindowSize"].get<int>();
    if (profileSettings.contains("speckleRange")) settings.speckleRange = profileSettings["speckleRange"].get<int>();
    if (profileSettings.contains("disp12MaxDiff")) settings.disp12MaxDiff = profileSettings["disp12MaxDiff"].get<int>();
    if (profileSettings.contains("minDisparity")) settings.minDisparity = profileSettings["minDisparity"].get<int>();
    if (profileSettings.contains("p1")) settings.p1 = profileSettings["p1"].get<int>();
    if (profileSettings.contains("p2")) settings.p2 = profileSettings["p2"].get<int>();
    if (profileSettings.contains("sgmMode")) settings.sgmMode = profileSettings["sgmMode"].get<int>();
    if (profileSettings.contains("maxDisparity")) settings.maxDisparity = profileSettings["maxDisparity"].get<int>();
    if (profileSettings.contains("confidenceThreshold")) settings.confidenceThreshold = profileSettings["confidenceThreshold"].get<int>();
    if (profileSettings.contains("confidenceType")) settings.confidenceType = profileSettings["confidenceType"].get<int>();
    if (profileSettings.contains("vpiQuality")) settings.vpiQuality = profileSettings["vpiQuality"].get<int>();
    if (profileSettings.contains("p2Alpha")) settings.p2Alpha = profileSettings["p2Alpha"].get<int>();
    if (profileSettings.contains("uniqueness")) settings.uniqueness = profileSettings["uniqueness"].get<float>();
    if (profileSettings.contains("numPasses")) settings.numPasses = profileSettings["numPasses"].get<int>();

    if (profileSettings.contains("zMax")) settings.zMax = profileSettings["zMax"].get<float>();
    if (profileSettings.contains("zMax")) settings.zMin = profileSettings["zMax"].get<float>();

    // Basic sanity (VPI CUDA stereo constraints).
    VisionControls::sanitizeVpiStereoSettings(settings);
    if (settings.quality < 1 || settings.quality > 100) settings.quality = 100;
    if (settings.frameRate < 1 || settings.frameRate > 120) settings.frameRate = 30;
    if (settings.preFilterSize < 5 || (settings.preFilterSize % 2) == 0) settings.preFilterSize = 9;
    if (settings.textureThreshold < 0) settings.textureThreshold = 10;

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO,
                                 "Loaded video streaming profile from: %s\n",
                                 StorageLocation);
    return 0;
}


/**
 * @brief Received frame callback
 * 
 * @param pbuf Pointer to UDP receive buffer
 * @param length Length of data received
 */
void VisionControls::onEthRecv(std::vector<char>& data) {
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
    Vision::VideoStreamer::VideoPacket packet;

    int ret = Vision::VideoStreamer::decodePacket(pbuf, packetSize, packet);
    if (ret != 0) {
        logger->log(Logger::LOG_LVL_WARN, "Frame %llu decode packet error %d\n", static_cast<unsigned long long>(seqId), ret);
        m_StreamInFrame.reset();
        return;
    }

    if (!packet.getVideoName().empty()) {
        m_LastIncomingVideoName = packet.getVideoName();
    }

    numSegments  = packet.getNumSegments();
    segmentID    = packet.getSegmentID();
    totalLength  = packet.getTotalLength();
    payloadLen   = packet.getPayloadLen();
    seqId        = packet.getSequenceID();
    segmentData  = packet.getPayload();    

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
            cv::Mat cvFrame;
            decodeJPEG(cvFrame, m_StreamInFrame);
            if (cvFrame.empty()) {
                logger->log(Logger::LOG_LVL_WARN,
                            "Frame %llu failed to decode JPEG\n",
                            static_cast<unsigned long long>(m_StreamInFrame.frameID()));
                m_StreamInFrame.reset();
                return;
            }
            m_VideoRecorder.pushFrame(cvFrame);
        }
        
        m_StreamInFrame.reset();
    } else if (m_StreamInFrame.numSegments() > numSegments) {
        // Should not happen due to earlier duplicate check
        logger->log(Logger::LOG_LVL_WARN,
                    "Frame %llu has excess segments (%zu / %u)\n",
                    static_cast<unsigned long long>(seqId),
                    m_StreamInFrame.numSegments(),
                    numSegments);
        m_StreamInFrame.reset();
        return;
    }
}


void VisionControls::decodeJPEG(cv::Mat& frame, const Vision::VideoFrame& frameEntry) {
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
 * @brief Process stereo and compiles a stereo frame pair
 * 
 * @param stereoFrame Input/output stereo frame pair
 */
void VisionControls::processStereo(cv::Mat& disparityFrame, cv::Mat& pointCloudMat, std::pair<cv::Mat, cv::Mat>& stereoFramePair, cv::Matx44d& Q) {
    cv::Mat& frameL = stereoFramePair.first;
    cv::Mat& frameR = stereoFramePair.second;

    static double sx = static_cast<double>(w_) / static_cast<double>(CAM_WIDTH);
    static double sy = static_cast<double>(h_) / static_cast<double>(CAM_HEIGHT);

    int ret = -1;
    cv::Mat cvImageLeft, cvImageRight;
    cv::Mat cvDisparity, cvDisparityu8, cvDisparityf32, cvDisparityColor, cvPointCloudMat;

    cv::cuda::Stream stream;
    cv::cuda::GpuMat d_disp32f, d_xyz;
    cv::Mat zMap(h_, w_, CV_32FC(6));

    Q = m_VideoCalib.reprojectionQ();

    // Scale Q Matrix
    Q(0, 3) *= sx;
    Q(1, 3) *= sy;
    Q(2, 3) *= sx;
    Q(3, 3) *= sx;    

    cv::Mat grayL, grayR;
    if (frameL.channels() == 1)      grayL = frameL;
    else if (frameL.channels() == 4) cv::cvtColor(frameL, grayL, cv::COLOR_BGRA2GRAY);
    else                             cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);

    if (frameR.channels() == 1)      grayR = frameR;
    else if (frameR.channels() == 4) cv::cvtColor(frameR, grayR, cv::COLOR_BGRA2GRAY);
    else                             cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);

    if (frameL.channels() == 4) {
        cv::cvtColor(frameL, cvImageLeft, cv::COLOR_BGRA2BGR);
    } else {
        cvImageLeft = frameL;
    }
    if (frameR.channels() == 4) {
        cv::cvtColor(frameR, cvImageRight, cv::COLOR_BGRA2BGR);
    } else {
        cvImageRight = frameR;
    }

    // rectify 
    cv::Mat rectLBuff, rectRBuff;
    cv::Mat rectL, rectR;
    m_VideoCalib.rectify(cvImageLeft, cvImageRight, rectL, rectR);

    vpiImageSetWrappedOpenCVMat(m_ImgL, rectL);
    vpiImageSetWrappedOpenCVMat(m_ImgR, rectR);

    // Convert to uint8
    CHECK_STATUS(vpiSubmitConvertImageFormat(m_VpiStream, VPI_BACKEND_CUDA, m_ImgL, m_ImgL_8u, &m_ConvParams));
    CHECK_STATUS(vpiSubmitConvertImageFormat(m_VpiStream, VPI_BACKEND_CUDA, m_ImgR, m_ImgR_8u, &m_ConvParams));

    // Rescale
    CHECK_STATUS(vpiSubmitRescale(m_VpiStream, VPI_BACKEND_CUDA, m_ImgL_8u, m_ImgL_270p, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
    CHECK_STATUS(vpiSubmitRescale(m_VpiStream, VPI_BACKEND_CUDA, m_ImgR_8u, m_ImgR_270p, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));

    // Disparity
    {
        std::lock_guard<std::mutex> guard(m_StereoMutex);
        sanitizeVpiStereoSettings(m_CamSettings);

        m_StereoParams.maxDisparity = m_CamSettings.maxDisparity;
        // m_StereoParams.confidenceThreshold = m_CamSettings.confidenceThreshold;
        // m_StereoParams.quality = m_CamSettings.vpiQuality;
        // m_StereoParams.confidenceType = static_cast<VPIStereoDisparityConfidenceType>(m_CamSettings.confidenceType);
        // m_StereoParams.minDisparity = m_CamSettings.minDisparity;
        m_StereoParams.confidenceThreshold = m_CamSettings.confidenceThreshold;
        m_StereoParams.p1 = m_CamSettings.p1;
        m_StereoParams.p2 = m_CamSettings.p2;
        // m_StereoParams.p2Alpha = m_CamSettings.p2Alpha;
        // m_StereoParams.uniqueness = m_CamSettings.uniqueness;
        // m_StereoParams.numPasses = static_cast<int8_t>(m_CamSettings.numPasses);
    }
    CHECK_STATUS(vpiSubmitStereoDisparityEstimator(m_VpiStream, VPI_BACKEND_CUDA, m_Stereo, m_ImgL_270p, m_ImgR_270p, m_Disparity, m_ConfidenceMap, &m_StereoParams));

    // Sync
    CHECK_STATUS(vpiStreamSync(m_VpiStream));

    // Lock disparity
    VPIImageData data;
    CHECK_STATUS(vpiImageLockData(m_Disparity, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data));

    // vpi image -> cv::Mat
    CHECK_STATUS(vpiImageDataExportOpenCVMat(data, &cvDisparity));

    // Q10.5 -> float
    cvDisparity.convertTo(cvDisparityu8, CV_8UC1, 255.0 / (32 * m_CreateParams.maxDisparity), 0);
    applyColorMap(cvDisparityu8, cvDisparityColor, cv::COLORMAP_JET);

    disparityFrame = cvDisparityColor;

    // Convert disparity to float mat
    cvDisparity.convertTo(cvDisparityf32, CV_32F, 1.0 / (32.0), 0);

    // Unlock disparity BEFORE any further VPI/CUDA work on m_Disparity
    CHECK_STATUS(vpiImageUnlock(m_Disparity));

    cv::Mat Q32;
    cv::Mat(Q).convertTo(Q32, CV_32F);

    // Convert to 3d_disp32f, d_xyz;
    // Allocate output with 3 channels
    d_disp32f.upload(cvDisparityf32, stream);

    d_xyz.create(d_disp32f.size(), CV_32FC3);
    cv::cuda::reprojectImageTo3D(d_disp32f, d_xyz, Q32, 3, stream);
    d_xyz.download(cvPointCloudMat, stream);
    stream.waitForCompletion();  // Sync

    {
        std::vector<cv::Mat> channels(3);
        cv::split(cvPointCloudMat, channels);
        cv::Mat X = channels[0], Y = channels[1], Z = channels[2];

        // Finite check (NaN != NaN)
        cv::Mat finite_mask = (X == X) & (Y == Y) & (Z == Z);

        // Filter out Inf
        cv::Mat abs_x, abs_y, abs_z;
        cv::absdiff(X, cv::Scalar(0), abs_x);
        cv::absdiff(Y, cv::Scalar(0), abs_y);
        cv::absdiff(Z, cv::Scalar(0), abs_z);
        finite_mask &= (abs_x < 1e10f) & (abs_y < 1e10f) & (abs_z < 1e10f);

        // Range filter
        cv::Mat pc_mask = finite_mask
            & (Z > m_CamSettings.zMin)
            & (Z < m_CamSettings.zMax)
            & (abs_x < 12.0f)
            & (abs_y < 8.0f);

        // Apply mask — zero out invalid points
        cv::Mat filtered = cv::Mat::zeros(cvPointCloudMat.size(), cvPointCloudMat.type());
        cvPointCloudMat.copyTo(filtered, pc_mask);

        // Fallback
        if (cv::countNonZero(pc_mask) == 0) {
            cvPointCloudMat.copyTo(filtered, finite_mask);
        }

        cvPointCloudMat = filtered;
    }

    // Resize the left colour image to match the point-cloud resolution
    cv::Mat colorResized;
    cv::resize(rectL, colorResized, cv::Size(w_, h_));

    // Fill in the XYZ coordinates and color
    ret = cuda::pointCloudToColor(cvPointCloudMat, colorResized, zMap);
    if (ret < 0) {
        printf("Failed convert point cloud\n");
    }

    pointCloudMat = zMap;
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
    if (!cmd) {
        return -1;
    }

    std::vector<char> responseBuffer;
    Logger* logger = Logger::getLoggerInst();
    auto cmdToString = [](uint8_t c) -> const char* {
        switch (c) {
            case VisionControls::CmdStartStream:            return "CmdStartStream";
            case VisionControls::CmdStopStream:             return "CmdStopStream";
            case VisionControls::CmdSelCameraStream:        return "CmdSelCameraStream";
            case VisionControls::CmdSetFps:                 return "CmdSetFps";
            case VisionControls::CmdSetQuality:             return "CmdSetQuality";
            case VisionControls::CmdSetUniquenessRatio:     return "CmdSetUniquenessRatio";
            case VisionControls::CmdSetMinDisparities:      return "CmdSetMinDisparities";
            case VisionControls::CmdSetMaxDisparities:      return "CmdSetMaxDisparities";
            case VisionControls::CmdSetConfidenceThreshold: return "CmdSetConfidenceThreshold";
            case VisionControls::CmdSetP1:                  return "CmdSetP1";
            case VisionControls::CmdSetP2:                  return "CmdSetP2";
            case VisionControls::CmdSetZMax:                return "CmdSetZMax";
            case VisionControls::CmdSetZMin:                return "CmdSetZMin";
            case VisionControls::CmdRdParams:               return "CmdRdParams";
            case VisionControls::CmdClrVideoRec:            return "CmdClrVideoRec";
            case VisionControls::CmdSaveVideo:              return "CmdSaveVideo";
            case VisionControls::CmdLoadStoredVideos:       return "CmdLoadStoredVideos";
            case VisionControls::CmdLoadSelectedVideo:      return "CmdLoadSelectedVideo";
            case VisionControls::CmdDeleteVideo:            return "CmdDeleteVideo";
            case VisionControls::CmdCalibrationSetState:    return "CmdCalibrationSetState";
            case VisionControls::CmdCalibrationWrtParams:   return "CmdCalibrationWrtParams";
            case VisionControls::CmdCalibrationReset:       return "CmdCalibrationReset";
            case VisionControls::CmdCalibrationSave:        return "CmdCalibrationSave";
            default: return "CmdUnknown";
        }
    };
    logger->log(Logger::LOG_LVL_INFO,
                "VisionControls received command: %s (%u), value[i32]=%.3f value[i32]=%d, value[u16]=%u, value[u8]=%u, payloadLen=%u\n",
                cmdToString(cmd->command),
                cmd->command,
                cmd->data.f32,
                cmd->data.i32,
                cmd->data.u16,
                cmd->data.u8,
                cmd->payloadLen);

    switch (cmd->command) 
    {
        case VisionControls::CmdStartStream:
            m_VideoStreamer->start();
            break;

        case VisionControls::CmdStopStream:
            m_VideoStreamer->stop();
            break;

        case VisionControls::CmdSelCameraStream: {
            m_CamSettings.streamSelection.store(cmd->data.u8);

            if (cmd->data.u8 == VisionControls::StreamCameraSource) {
                m_VideoRecorder.resetPlayback();
                logger->log(Logger::LOG_LVL_INFO, "Selecting normal camera mode\n");
                char* selModeJson = reinterpret_cast<char*>(buffer.data() + sizeof(VisionControls::CameraCommand));
                selModeJson[cmd->payloadLen] = '\0';
                nlohmann::json jsonParams;
                try {
                    jsonParams = nlohmann::json::parse(std::string(selModeJson));
                } catch (nlohmann::json::parse_error& e) {
                    logger->log(Logger::LOG_LVL_ERROR, "Failed to parse camera stream selection JSON: %s\n", e.what());
                    return -1;
                }

                if (jsonParams.contains("calibration-mode")) {
                    m_CamSettings.calibrationMode = jsonParams["calibration-mode"].get<bool>();
                }

                if (m_CamSettings.calibrationMode.load()) {
                    logger->log(Logger::LOG_LVL_INFO, "Camera calibration mode enabled\n");
                } else {
                    logger->log(Logger::LOG_LVL_INFO, "Camera calibration mode disabled\n");
                }
                return 0;
            } else if (cmd->data.u8 == VisionControls::StreamSimSource) {
                logger->log(Logger::LOG_LVL_INFO, "Selecting training mode\n");
            } else {
                logger->log(Logger::LOG_LVL_ERROR, "Invalid camera stream selection: %d\n", cmd->data.u8);
                return -1;
            }

            break;
        }

        case VisionControls::CmdSetFps: {
            int ret = m_VideoStreamer->setStreamFrameRate(static_cast<Vision::VideoStreamer::FrameRate>(cmd->data.u8));
            if (ret < 0) {
                logger->log(Logger::LOG_LVL_ERROR, "Failed to set FPS throttle to %u fps\n", cmd->data.u8);
                return -1;
            }
            m_CamSettings.frameRate = cmd->data.u8;
            logger->log(Logger::LOG_LVL_INFO, "Set stream FPS throttle to %u fps\n", cmd->data.u8);
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdRdParams: {
            nlohmann::json jsonResponse;
            jsonResponse["quality"]     = m_VideoStreamer->getQuality();
            jsonResponse["fps"]         = m_VideoStreamer->getFrameRate();
            jsonResponse["uniquenessRatio"]  = m_CamSettings.uniquenessRatio;
            jsonResponse["minDisparity"]     = m_CamSettings.minDisparity;
            jsonResponse["p1"]               = m_CamSettings.p1;
            jsonResponse["p2"]               = m_CamSettings.p2;
            jsonResponse["maxDisparity"]        = m_CamSettings.maxDisparity;
            jsonResponse["confidenceThreshold"] = m_CamSettings.confidenceThreshold;
            jsonResponse["confidenceType"]      = m_CamSettings.confidenceType;
            jsonResponse["vpiQuality"]          = m_CamSettings.vpiQuality;
            jsonResponse["p2Alpha"]             = m_CamSettings.p2Alpha;
            jsonResponse["uniqueness"]          = m_CamSettings.uniqueness;
            jsonResponse["numPasses"]           = m_CamSettings.numPasses;
            jsonResponse["zMax"]                = m_CamSettings.zMax;
            jsonResponse["zMin"]                = m_CamSettings.zMin;
            responseBuffer.resize(jsonResponse.dump().size());
            std::strncpy(responseBuffer.data(), jsonResponse.dump().c_str(), jsonResponse.dump().size());
            break;;
        }

        case VisionControls::CmdSetQuality: {
            int ret = m_VideoStreamer->setJpegQuality(cmd->data.i32);
            if (ret < 0) {
                logger->log(Logger::LOG_LVL_ERROR, "Failed to set quality: %d\n", cmd->data.u8);
            }

            logger->log(Logger::LOG_LVL_INFO, "Set quality to %u\n", cmd->data.u8);
            m_CamSettings.quality = cmd->data.u8;
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetMinDisparities: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            m_CamSettings.minDisparity = clampInt(cmd->data.i32, 0, m_CamSettings.numDisparities);
            sanitizeVpiStereoSettings(m_CamSettings);
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetMaxDisparities: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            m_CamSettings.maxDisparity = clampInt(cmd->data.i32, 0, m_CamSettings.numDisparities);
            sanitizeVpiStereoSettings(m_CamSettings);
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetConfidenceThreshold: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            m_CamSettings.confidenceThreshold = clampInt(cmd->data.u32, 0, 65535);
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetP1: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            m_CamSettings.p1 = clampInt(cmd->data.i32, 1, 255);
            if (m_CamSettings.p2 < m_CamSettings.p1) m_CamSettings.p2 = m_CamSettings.p1;
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetP2: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            m_CamSettings.p2 = clampInt(cmd->data.i32, 1, 255);
            if (m_CamSettings.p2 < m_CamSettings.p1) m_CamSettings.p2 = m_CamSettings.p1;
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetZMax: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            float setting = cmd->data.f32; 
            if (setting < m_CamSettings.zMin) {
                setting = m_CamSettings.zMax;
            }
            m_CamSettings.zMax = cmd->data.f32;
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetZMin: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            float setting = cmd->data.f32; 
            if (setting > m_CamSettings.zMax) {
                setting = m_CamSettings.zMax;
            }
            m_CamSettings.zMin = cmd->data.f32;
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetUniquenessRatio: {
            std::lock_guard<std::mutex> guard(m_StereoMutex);
            m_CamSettings.uniquenessRatio = clampInt(cmd->data.i32, 0, 30);
            m_CamSettings.uniqueness = static_cast<float>(m_CamSettings.uniquenessRatio) / 100.0f;
            VisionControls::saveStreamingProfile(m_CamSettings);
        break;
        }

        case VisionControls::CmdClrVideoRec:
            logger->log(Logger::LOG_LVL_INFO, "Clearing video recording buffer\n");
            m_StreamInFrame.reset();
            m_VideoRecorder.clear();
            m_LastIncomingVideoName.clear();
            break;

        case VisionControls::CmdSaveVideo: {
            std::string nameToSave = m_CamSettings.videoName;
            // If no explicit name was ever set, prefer the incoming metadata name (downloaded file).
            if (nameToSave == "recording.MOV" && !m_LastIncomingVideoName.empty()) {
                nameToSave = m_LastIncomingVideoName;
            }

            logger->log(Logger::LOG_LVL_INFO, "Saving video recording to file: %s\n", nameToSave.c_str());
            if (m_VideoRecorder.saveToFile(nameToSave) < 0) {
                logger->log(Logger::LOG_LVL_WARN, "Failed to save video recording to file: %s\n", nameToSave.c_str());
                return -1;
            }

            break;
        }

        case VisionControls::CmdLoadStoredVideos: {
            logger->log(Logger::LOG_LVL_INFO, "Loading stored videos from disk\n");
            std::vector<std::string> videoFiles = m_VideoRecorder.listRecordedFiles();
            std::stringstream ss;
            for (const auto& file : videoFiles) {
                std::cout << "Found video file: " << file << std::endl;
                ss << file << ";";
            }
            nlohmann::json jsonResponse;
            std::cout << "Videos: " << ss.str() << std::endl;
            jsonResponse["loaded-video"] = m_VideoRecorder.getLoadedVideoName();
            jsonResponse["video-list"] = ss.str();
            responseBuffer.resize(jsonResponse.dump().size());
            std::strncpy(responseBuffer.data(), jsonResponse.dump().c_str(), jsonResponse.dump().size());
            break;
        }

        case VisionControls::CmdLoadSelectedVideo: {
            char* namePtr = reinterpret_cast<char*>(buffer.data() + sizeof(VisionControls::CameraCommand));
            if (namePtr[cmd->payloadLen] != '\0' && strlen(namePtr) >= cmd->payloadLen) {
                logger->log(Logger::LOG_LVL_ERROR, "Invalid video name\n");
                return -1;
            }

            int ret = m_VideoRecorder.loadFile(std::string(namePtr));
            if (ret < 0) {
                logger->log(Logger::LOG_LVL_ERROR, "Failed to load video: %s\n", namePtr);
                return -1;
            }
            
            logger->log(Logger::LOG_LVL_INFO, "Loading selected video from disk: %s\n", std::string(namePtr, cmd->payloadLen).c_str());
            break;
        }

        case VisionControls::CmdDeleteVideo: {
            char* namePtr = reinterpret_cast<char*>(buffer.data() + sizeof(VisionControls::CameraCommand));
            if (namePtr[cmd->payloadLen] != '\0' && strlen(namePtr) >= cmd->payloadLen) {
                logger->log(Logger::LOG_LVL_ERROR, "Invalid video name\n");
                return -1;
            }

            int ret = m_VideoRecorder.deleteVideo(std::string(namePtr));
            if (ret < 0) {
                logger->log(Logger::LOG_LVL_ERROR, "Failed to delete video: %s\n", namePtr);
                return -1;
            }

            logger->log(Logger::LOG_LVL_INFO, "Deleted video from disk: %s\n", std::string(namePtr, cmd->payloadLen).c_str());
            break;
        }

        case VisionControls::CmdCalibrationSetState:
            break;

        case VisionControls::CmdCalibrationWrtParams: {
            char* calibrationParams = reinterpret_cast<char*>(buffer.data() + sizeof(VisionControls::CameraCommand));
            calibrationParams[cmd->payloadLen] = '\0';
            logger->log(Logger::LOG_LVL_INFO, "Received parameters: %s\n", calibrationParams);

            int ret = m_VideoCalib.configureFromJson(std::string(calibrationParams));
            if (ret < 0) {
                logger->log(Logger::LOG_LVL_ERROR, "Failed to configure calibration parameters from JSON\n");
                return -1;
            }
            break;
        }

        case VisionControls::CmdCalibrationReset: {
            logger->log(Logger::LOG_LVL_INFO, "Resetting calibration parameters to default\n");
            m_VideoCalib.resetCalibrationSession();
            break;
        }

        case VisionControls::CmdCalibrationSave: {
            logger->log(Logger::LOG_LVL_INFO, "Saving calibration parameters to disk\n");
            int ret = m_VideoCalib.storeCalibrationProfile();
            if (ret < 0) {
                logger->log(Logger::LOG_LVL_ERROR, "Failed to save calibration parameters to disk\n");
                return -1;
            }
            break;
        }

        default:
            logger->log(Logger::LOG_LVL_ERROR, "Remote command not recognized: %d\n", cmd->command);
            return -1;
    }

    if (!responseBuffer.empty()) {
        buffer = responseBuffer;
    } else {
        buffer.clear();
    }
    return 0;
}


std::string VisionControls::readStats() {
    std::stringstream ss;

    ss << "Camera stats: " << "\r\n" << m_VideoCalib.getCurrentCalibrationStats().dump(4) << "\r\n"
         << "Video stream stats: " << "\r\n" << m_Cam->getTimestampDiffNs() << " nanoseconds" << "\r\n";
    
    return ss.str();
}


/**
 * @brief Thread timer handler
 * 
 */
void VisionControls::OnTimer(void) {
    if (m_CamSettings.calibrationMode.load()) {
        nlohmann::json calibStats = m_VideoCalib.getCurrentCalibrationStats();
        std::string statsStr = calibStats.dump();
        this->TlmAdapter->publishTelemetry(
            this->getName(),
            reinterpret_cast<const uint8_t*>(statsStr.c_str()),
            statsStr.size());
        return;
    }
}


void VisionControls::mainProc() {
    Logger* logger = Logger::getLoggerInst();

    // Devices::StereoCam cam(0, 1);
    cv::Mat frameL;
    cv::Mat frameR;
    cv::Mat frameStereo;

    int ret = -1;
    cv::Mat frameSim;
    cv::Mat resizedFrame;

    const int _resizeWidth  = 1280;
    const int _resizeHeight = 720;

    std::pair<cv::Mat, cv::Mat> stereoFramePair;
    int16_t xAccel, yAccel, zAccel;
    int16_t xGyro, yGyro, zGyro;
    cv::Matx44d Q;
    
    cv::Mat frameOut;

    auto scaleRectMat = [this, _resizeWidth, _resizeHeight](cv::Mat& frame, cv::Mat& resizedFrame, cv::Matx44d& Q) -> void {
        if (frame.empty()) {
            resizedFrame.release();
            return;
        }

        const int dstW = _resizeWidth;
        const int dstH = _resizeHeight;
        const int srcW = frame.cols;
        const int srcH = frame.rows;
        if (dstW <= 0 || dstH <= 0 || srcW <= 0 || srcH <= 0) {
            resizedFrame = frame;
            return;
        }

        const double sx = static_cast<double>(dstW) / static_cast<double>(srcW);
        const double sy = static_cast<double>(dstH) / static_cast<double>(srcH);

        const int interp = (frame.depth() == CV_16U || frame.depth() == CV_16S) ? cv::INTER_NEAREST : cv::INTER_LINEAR;
        cv::resize(frame, resizedFrame, cv::Size(dstW, dstH), 0.0, 0.0, interp);

        // If we're resizing an already-computed disparity map, disparity must scale with x-resolution.
        // m_stereoBM outputs fixed-point disparity (d*16).
        if ((frame.type() == CV_16U || frame.type() == CV_16S) && resizedFrame.type() == frame.type()) {
            cv::Mat scaled;
            resizedFrame.convertTo(scaled, CV_32F);
            scaled *= static_cast<float>(sx);
            scaled.convertTo(resizedFrame, resizedFrame.type());
        }

        // Scale pixel-dependent terms of Q to match the transmitted resolution.
        // Standard OpenCV Q has: Q(0,3)=-cx, Q(1,3)=-cy, Q(2,3)=f, Q(3,2)=1/Tx, Q(3,3)=(cx-cx')/Tx.
        Q(0, 3) *= sx;
        Q(1, 3) *= sy;
        Q(2, 3) *= sx;
        Q(3, 3) *= sx;
    };


    // Create VPI image wrappers (require a non-empty frame from the camera).
    cv::Mat cvImageLeft, cvImageRight;

    for (int i = 0; i < 5; i++) {
        // Read once so wrappers are created from non-empty mats (matches test2 flow).
        ret = m_Cam->read(frameL, frameR, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);
        if (ret == 0) {
            break;
        }

        std::cout << "Attempt " << i << std::endl;
        logger->log(Logger::LOG_LVL_INFO, "Camera attempt %d of 5\r\n", i);
        if (i == 5) {
            logger->log(Logger::LOG_LVL_ERROR, "CamFailed to open camera\r\n");
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (frameL.channels() == 4) {
        cv::cvtColor(frameL, cvImageLeft, cv::COLOR_BGRA2BGR);
    } else {
        cvImageLeft = frameL;
    }

    if (frameR.channels() == 4) {
        cv::cvtColor(frameR, cvImageRight, cv::COLOR_BGRA2BGR);
    } else {
        cvImageRight = frameR;
    }

    // Wrap cv::Mat in vpiImage
    CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvImageLeft, VPI_IMAGE_FORMAT_BGR8, VPI_BACKEND_CUDA, &m_ImgL));
    CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvImageRight, VPI_IMAGE_FORMAT_BGR8, VPI_BACKEND_CUDA, &m_ImgR));

    // Initialise reprojection matrix
    Q = m_VideoCalib.reprojectionQ();

    while (m_Running.load()) {
        switch(m_CamSettings.streamSelection.load()) {
            case VisionControls::StreamCameraSource:
                ret = m_Cam->read(frameL, frameR, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);
                if (ret != 0) {
                    continue;
                }

                stereoFramePair = std::make_pair(frameL, frameR);
                if (m_CamSettings.calibrationMode.load()) {
                    m_VideoCalib.DoCalibration(frameL, frameR);

                    if (!frameL.empty() && !frameR.empty()) {
                        cv::hconcat(frameL, frameR, frameOut);
                        // Change to grayscale
                        auto Q = m_VideoCalib.reprojectionQ();
                        m_VideoStreamer->pushFrame(frameOut, Q);
                    }
                } else {
                    cv::Mat pointCloud;

                    // Extract the disparity frame
                    processStereo(frameStereo, pointCloud, stereoFramePair, Q);             

                    // Select the frame base on the ethernet link status
                    if (m_EthAdapter->ethLinkDetected.load()) {
                        frameOut = pointCloud;
                    } else {                        
                        // Resize disparity before transmission and scale Q accordingly.
                        scaleRectMat(frameStereo, resizedFrame, Q);
                        frameOut = resizedFrame;  // Reference the frame out to the resized frame
                    }

                    // Stream disparity frame
                    m_VideoStreamer->pushFrame(frameOut, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel, Q);
                }
                break;

            case VisionControls::StreamSimSource: {
                // We draw a frame from the recording object
                cv::Mat simFrame = m_VideoRecorder.getNextFrame();
                if (simFrame.empty()) {
                    // No frames available, wait a bit
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                frameSim = simFrame;
                m_VideoStreamer->pushFrame(frameSim);
                break;
            }

            default:
                break;
        }
    }
}
}
