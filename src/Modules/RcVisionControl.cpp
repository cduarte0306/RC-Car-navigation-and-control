#include "RcVisionControl.hpp"
#include "RcMessageLib.hpp"
#include "app/video/VideoStreamer.hpp"
#include "app/video/VideoFrame.hpp"
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

#define STREAM_PORT 5005

#define MODEL_PATH  "/opt/rc-car/models/model-small.onnx"


const char* StorageLocation = "/data/calibration-data/streaming-profile.json";

static cv::Ptr<cv::cuda::StereoBM> stereoBM = nullptr;
static cv::cuda::GpuMat d_left, d_right, d_disp;
static cv::cuda::Stream stream;

namespace Modules {
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
    m_TxAdapter = this->CommsAdapter->createNetworkAdapter(STREAM_PORT, "wlP1p1s0", Adapter::CommsAdapter::MaxUDPPacketSize);
    m_RxAdapter = this->CommsAdapter->createNetworkAdapter(STREAM_PORT, "enP8p1s0", Adapter::CommsAdapter::MaxUDPPacketSize);
    if (!m_TxAdapter || !m_RxAdapter) {
        logger->log(Logger::LOG_LVL_WARN, "Failed to create vision network adapters\r\n");
        return -1;
    }

    // Create the streamer only once the TX adapter exists; otherwise we'd bind a dangling reference.
    m_VideoStreamer = std::make_unique<Vision::VideoStreamer>(*m_TxAdapter, 100);

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
    stereoBM = cv::cuda::createStereoBM(m_CamSettings.numDisparities, m_CamSettings.numBlocks);
    m_NumDisparities = 96;

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

    // Basic sanity
    if (settings.numDisparities < 8 || (settings.numDisparities % 8) != 0) settings.numDisparities = 96;
    if (settings.numBlocks < 5 || (settings.numBlocks % 2) == 0) settings.numBlocks = 15;
    if (settings.quality < 1 || settings.quality > 100) settings.quality = 100;
    if (settings.frameRate < 1 || settings.frameRate > 120) settings.frameRate = 30;

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
void VisionControls::processStereo(cv::Mat& stereoFrame, std::pair<cv::Mat, cv::Mat>& stereoFramePair, cv::Matx44d& Q) {
    using namespace std;
    using namespace cv;
    using namespace dnn;

    cv::Mat& frameL = stereoFramePair.first;
    cv::Mat& frameR = stereoFramePair.second;

    // Build grayscale inputs robustly (StereoCam frames can be BGRA).
    cv::Mat grayL, grayR;
    if (frameL.channels() == 1) {
        grayL = frameL;
    } else if (frameL.channels() == 4) {
        cv::cvtColor(frameL, grayL, cv::COLOR_BGRA2GRAY);
    } else {
        cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
    }

    if (frameR.channels() == 1) {
        grayR = frameR;
    } else if (frameR.channels() == 4) {
        cv::cvtColor(frameR, grayR, cv::COLOR_BGRA2GRAY);
    } else {
        cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);
    }

    // Apply calibration before disparity: undistort + rectify into rectified grayscale frames.
    cv::Mat rectL, rectR;
    m_VideoCalib.rectify(grayL, grayR, rectL, rectR);

    cv::Mat disparity;
    // cv::cuda::* algorithms require cuda::GpuMat/HostMem. Passing cv::Mat triggers
    // InputArray::getGpuMat() and throws "getGpuMat is available only for cuda::GpuMat".
    d_left.upload(rectL, stream);
    d_right.upload(rectR, stream);
    std::lock_guard<std::mutex> guard(m_StereoMutex);

    try {
        stereoBM->compute(d_left, d_right, d_disp, stream);
    } catch (cv::Exception& e) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Stereo conversion failed: %s\n", e.what());
        m_CamSettings.numDisparities = 96;
        stereoBM.reset();
        stereoBM = cv::cuda::createStereoBM(m_CamSettings.numDisparities, m_CamSettings.numBlocks);
    }

    d_disp.download(disparity, stream);
    stream.waitForCompletion();

    if (disparity.empty()) {
        stereoFrame.release();
        return;
    }

    // Send raw disparity as 16-bit single-channel PNG-friendly format.
    // StereoBM outputs fixed-point disparity (typically scaled by 16) and may be signed.
    if (disparity.type() == CV_16U) {
        stereoFrame = disparity;
        return;
    }

    cv::Mat disp16s;
    int type = disparity.type();
    if (type == CV_16S) {
        disp16s = disparity;
    } else {
        disparity.convertTo(disp16s, CV_16S);
    }

    cv::Mat dispClamped16s;
    cv::max(disp16s, 0, dispClamped16s);

    cv::Mat disp16u;
    dispClamped16s.convertTo(disp16u, CV_16U);

    stereoFrame = disp16u;

    // Read the reprojection matrix
    Q = m_VideoCalib.reprojectionQ();
}


/**
 * @brief Process disparity frame into point cloud
 * 
 * @param dispFrame 
 */
void VisionControls::doPointCloud(cv::Mat& dispFrame, cv::Matx44d& Q) {
    // Q is the reprojection matrix
    cv::Mat Q64(Q);   // CV_64F 4x4
    cv::Mat Q32;
    Q64.convertTo(Q32, CV_32F);

    cv::Mat disp32f;
    dispFrame.convertTo(disp32f, CV_32F, 1.0 / 16.0);

    cv::Mat points3d;
    cv::cuda::GpuMat d_disparity, d_3dImage;
    d_disparity.upload(disp32f);
    cv::cuda::reprojectImageTo3D(d_disparity, d_3dImage, Q32, 3, stream);
    d_3dImage.download(points3d);
    stream.waitForCompletion();
    
    // Convert floating point to uint16_t    
    int x = points3d.cols / 2;
    int y = points3d.rows / 2;
    cv::Vec3f xyz = points3d.at<cv::Vec3f>(y, x);
    float Z = xyz[2]; // meters (if your calibration square size was meters)
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
        case VisionControls::CamModeNormal:
            /* code */
            break;

        case VisionControls::CamModeDisparity:
            
            break;

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
    if (!cmd) {
        return -1;
    }

    std::vector<char> responseBuffer;
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "VisionControls received command: %d\n", cmd->command);

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
            jsonResponse["disparities"] = m_CamSettings.numDisparities;
            jsonResponse["blocks"]      = m_CamSettings.numBlocks;
            responseBuffer.resize(jsonResponse.dump().size());
            std::strncpy(responseBuffer.data(), jsonResponse.dump().c_str(), jsonResponse.dump().size());
            break;;
        }

        case VisionControls::CmdSetQuality: {
            int ret = m_VideoStreamer->setJpegQuality(cmd->data.u8);
            if (ret < 0) {
                logger->log(Logger::LOG_LVL_ERROR, "Failed to set quality: %d\n", cmd->data.u8);
            }

            logger->log(Logger::LOG_LVL_INFO, "Set quality to %u\n", cmd->data.u8);
            m_CamSettings.quality = cmd->data.u8;
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetNumDisparities: {
            uint8_t numDisparities = cmd->data.u8;

            if (numDisparities < 8 || (numDisparities % 8) != 0) {
                logger->log(Logger::LOG_LVL_ERROR,
                            "Invalid disparity value %u (must be >= 8 and divisible by 8)\n",
                            numDisparities);
                return -1;
            }

            std::lock_guard<std::mutex> guard(m_StereoMutex);
            stereoBM.reset();
            stereoBM = cv::cuda::createStereoBM(numDisparities, m_CamSettings.numBlocks);
            m_CamSettings.numDisparities = numDisparities;
            logger->log(Logger::LOG_LVL_INFO, "Setting number of disparities to %u\n", numDisparities);
            VisionControls::saveStreamingProfile(m_CamSettings);
            break;
        }

        case VisionControls::CmdSetBlockSize: {
            const uint8_t blockSize = cmd->data.u8;
            if (blockSize < 5 || (blockSize % 2) == 0) {
                logger->log(Logger::LOG_LVL_ERROR,
                            "Invalid block size %u (must be odd and >= 5)\n",
                            blockSize);
                return -1;
            }

            std::lock_guard<std::mutex> guard(m_StereoMutex);
            stereoBM.reset();
            stereoBM = cv::cuda::createStereoBM(m_CamSettings.numDisparities, blockSize);
            m_CamSettings.numBlocks = blockSize;
            logger->log(Logger::LOG_LVL_INFO, "Setting block size to %u\n", blockSize);
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

        case VisionControls::CmdReadStoredVideos: {
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

    const int _resizeWidth  = 640;
    const int _resizeHeight = 360;

    std::pair<cv::Mat, cv::Mat> stereoFramePair;
    int16_t xAccel, yAccel, zAccel;
    int16_t xGyro, yGyro, zGyro;
    cv::Matx44d Q;

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
        // StereoBM outputs fixed-point disparity (d*16).
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
                    m_VideoStreamer->pushFrame(stereoFramePair);
                } else {
                    // Extract the disparity frame
                    processStereo(frameStereo, stereoFramePair, Q);

                    // Convert to point cloud for local processing
                    doPointCloud(frameStereo, Q);

                    // Resize disparity before transmission and scale Q accordingly.
                    scaleRectMat(frameStereo, resizedFrame, Q);

                    // Stream disparity frame
                    m_VideoStreamer->pushFrame(resizedFrame, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel, Q);
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
                processFrame(frameSim);
                m_VideoStreamer->pushFrame(frameSim);
                break;
            }

            default:
                break;
        }
    }
}
}
