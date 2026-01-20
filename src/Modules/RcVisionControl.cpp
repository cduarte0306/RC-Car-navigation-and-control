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


static cv::Ptr<cv::cuda::StereoBM> stereoBM = nullptr;
static cv::cuda::GpuMat d_left, d_right, d_disp;
static cv::cuda::Stream stream;

namespace Modules {
VisionControls::VisionControls(int moduleID, std::string name) :
    Base(moduleID, name), Adapter::CameraAdapter(name) {
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

    m_VideoStreamer->setJpegQuality(35);  // Default to maximum quality
    m_VideoStreamer->setStreamFrameRate(Vision::VideoStreamer::FrameRate::_30Fps);
    m_VideoStreamer->start();
    
    m_VideoRecorder.setFrameRate(Vision::VideoRecording::FrameRate::_30Fps);
    stereoBM = cv::cuda::createStereoBM(96, 15);

    logger->log(Logger::LOG_LVL_INFO, "Vision control module initialized\r\n");
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
void VisionControls::processStereo(cv::Mat& stereoFrame, std::pair<cv::Mat, cv::Mat>& stereoFramePair) {
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
    stereoBM->compute(d_left, d_right, d_disp, stream);
    d_disp.download(disparity, stream);
    stream.waitForCompletion();

    // Normalize for visualization.
    // StereoBM outputs fixed-point disparity (typically scaled by 16). If most values
    // are <= 0 (common when not rectified), the colormap will look mostly blue.
    cv::Mat disp32f;
    disparity.convertTo(disp32f, CV_32F, 1.0 / 16.0);
    cv::max(disp32f, 0.0f, disp32f);

    // Estimate distance at the image center using the rectified projection (meters).
    std::string depthText = "Z: --";
    {
        const int cx = disp32f.cols / 2;
        const int cy = disp32f.rows / 2;
        const int half = 5; // 11x11 window
        const int x0 = std::max(0, cx - half);
        const int y0 = std::max(0, cy - half);
        const int x1 = std::min(disp32f.cols - 1, cx + half);
        const int y1 = std::min(disp32f.rows - 1, cy + half);

        std::vector<float> vals;
        vals.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
        for (int y = y0; y <= y1; ++y) {
            const float* row = disp32f.ptr<float>(y);
            for (int x = x0; x <= x1; ++x) {
                const float d = row[x];
                if (d > 0.5f && std::isfinite(d)) vals.push_back(d);
            }
        }
        if (!vals.empty()) {
            const size_t mid = vals.size() / 2;
            std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
            const double dispPx = static_cast<double>(vals[mid]);
            if (auto z = m_VideoCalib.depthMetersFromDisparity(dispPx)) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "Z: %.2f m", *z);
                depthText = buf;
            }
        }
    }

    // Normalize for display using only valid pixels (otherwise the sea of zeros crushes contrast).
    // Also use percentile clipping so a handful of outliers/speckles don't force most surfaces
    // into dark blue.
    cv::Mat disp8U(disp32f.size(), CV_8U, cv::Scalar(0));
    cv::Mat validMask = disp32f > 0.0f;
    const int validCount = cv::countNonZero(validMask);
    if (validCount > 0) {
        std::vector<float> validVals;
        validVals.reserve(static_cast<size_t>(validCount));
        for (int y = 0; y < disp32f.rows; ++y) {
            const float* row = disp32f.ptr<float>(y);
            for (int x = 0; x < disp32f.cols; ++x) {
                const float d = row[x];
                if (d > 0.0f && std::isfinite(d)) validVals.push_back(d);
            }
        }

        if (validVals.size() >= 16) {
            const auto nth = [&](double p) -> float {
                const size_t idx = static_cast<size_t>(p * static_cast<double>(validVals.size() - 1));
                std::nth_element(validVals.begin(), validVals.begin() + idx, validVals.end());
                return validVals[idx];
            };

            // Clip to a robust range to avoid outliers dominating the colormap.
            const float lo = nth(0.05);   // 5th percentile
            const float hi = nth(0.95);   // 95th percentile
            if (hi > lo) {
                cv::Mat clipped;
                cv::min(cv::max(disp32f, lo), hi, clipped);

                const double scale = 255.0 / static_cast<double>(hi - lo);
                const double shift = -static_cast<double>(lo) * scale;
                cv::Mat scaled;
                clipped.convertTo(scaled, CV_8U, scale, shift);
                scaled.copyTo(disp8U, validMask);
            }
        } else {
            // Fall back to min/max when we don't have enough valid pixels.
            double vmin = 0.0, vmax = 0.0;
            cv::minMaxLoc(disp32f, &vmin, &vmax, nullptr, nullptr, validMask);
            if (vmax > vmin) {
                const double scale = 255.0 / (vmax - vmin);
                const double shift = -vmin * scale;
                cv::Mat scaled;
                disp32f.convertTo(scaled, CV_8U, scale, shift);
                scaled.copyTo(disp8U, validMask);
            }
        }
    }
    cv::applyColorMap(disp8U, stereoFrame, cv::COLORMAP_JET);

    cv::putText(stereoFrame, depthText, {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2, cv::LINE_AA);

    // Lightweight periodic debug to catch "all blue" (near-zero disparity) quickly.
    static auto lastLog = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (now - lastLog > std::chrono::seconds(2)) {
        double minVal = 0.0, maxVal = 0.0;
        cv::minMaxLoc(disp32f, &minVal, &maxVal);
        Logger::getLoggerInst()->log(Logger::LOG_LVL_DEBUG,
                                    "Stereo disparity range: min=%.2f max=%.2f (post-clamp)\n",
                                    minVal, maxVal);
        lastLog = now;
    }
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
        case VisionControls::CmdSetFrameRate:
            /* code */
            break;

        case VisionControls::CmdStartStream:
            /* code */
            break;

        case VisionControls::CmdStopStream:
            /* code */
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

            return 0;
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
            return 0;
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
            return 0;
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

    Devices::StereoCam cam(0, 1);
    cam.start(1920, 1080, 30);
    cv::Mat frameL;
    cv::Mat frameR;
    cv::Mat frameStereo;

    int ret = -1;
    cv::Mat frameSim;
    std::pair<cv::Mat, cv::Mat> stereoFramePair;
    int16_t xAccel, yAccel, zAccel;
    int16_t xGyro, yGyro, zGyro;
    while (m_Running.load()) {
        switch(m_CamSettings.streamSelection.load()) {
            case StreamCameraSource:
                ret = cam.read(frameL, frameR, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);
                if (ret != 0) {
                    continue;
                }

                stereoFramePair = std::make_pair(frameL, frameR);
                if (m_CamSettings.calibrationMode.load()) {
                    m_VideoCalib.DoCalibration(frameL, frameR);
                    m_VideoStreamer->pushFrame(stereoFramePair);
                } else {
                    processStereo(frameStereo, stereoFramePair);
                    m_VideoStreamer->pushFrame(frameStereo, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);
                }
                break;

            case StreamSimSource: {
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
