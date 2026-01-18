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

    // Load video into recording
    // m_VideoRecorder.loadFile();

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

    switch (m_CamSettings.mode) {
        case VisionControls::CamModeNormal:
            // For testing purposes. We just superimpose the two frames into one
            cv::addWeighted(frameL, 0.5, frameR, 0.5, 0.0, stereoFrame);
            break;

        case VisionControls::CamModeDisparity:
            {
                // Ensure grayscale inputs are correct (StereoCam frames can be BGRA).
                cv::Mat grayL, grayR;
                cv::Mat bgrL, bgrR;
                if (frameL.channels() == 4) {
                    cv::cvtColor(frameL, bgrL, cv::COLOR_BGRA2BGR);
                } else {
                    bgrL = frameL;
                }
                if (frameR.channels() == 4) {
                    cv::cvtColor(frameR, bgrR, cv::COLOR_BGRA2BGR);
                } else {
                    bgrR = frameR;
                }
                if (bgrL.channels() == 1) {
                    grayL = bgrL;
                } else {
                    cv::cvtColor(bgrL, grayL, cv::COLOR_BGR2GRAY);
                }
                if (bgrR.channels() == 1) {
                    grayR = bgrR;
                } else {
                    cv::cvtColor(bgrR, grayR, cv::COLOR_BGR2GRAY);
                }
#if RCVC_HAVE_CUDASTEREO
                static cv::Ptr<cv::cuda::StereoBM> stereoBM = cv::cuda::createStereoBM(96, 15);
                static cv::cuda::GpuMat d_left, d_right, d_disp;
                static cv::cuda::Stream stream;
#else
                static cv::Ptr<cv::StereoBM> stereoBM = cv::StereoBM::create(96, 15);
#endif
                cv::Mat disparity;
#if RCVC_HAVE_CUDASTEREO
                // cv::cuda::* algorithms require cuda::GpuMat/HostMem. Passing cv::Mat triggers
                // InputArray::getGpuMat() and throws "getGpuMat is available only for cuda::GpuMat".
                if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                    d_left.upload(grayL, stream);
                    d_right.upload(grayR, stream);
                    stereoBM->compute(d_left, d_right, d_disp, stream);
                    d_disp.download(disparity, stream);
                    stream.waitForCompletion();
                } else {
                    // Fallback to CPU if CUDA isn't available at runtime.
                    cv::Ptr<cv::StereoBM> cpuBM = cv::StereoBM::create(96, 15);
                    cpuBM->compute(grayL, grayR, disparity);
                }
#else
                stereoBM->compute(grayL, grayR, disparity);
#endif
                // Normalize for visualization.
                // StereoBM outputs fixed-point disparity (typically scaled by 16). If most values
                // are <= 0 (common when not rectified), the colormap will look mostly blue.
                cv::Mat disp32f;
                disparity.convertTo(disp32f, CV_32F, 1.0 / 16.0);
                cv::max(disp32f, 0.0f, disp32f);

                cv::Mat disp8U;
                cv::normalize(disp32f, disp8U, 0, 255, cv::NORM_MINMAX, CV_8U);
                cv::applyColorMap(disp8U, stereoFrame, cv::COLORMAP_JET);

                // Lightweight periodic debug to catch "all blue" (near-zero disparity) quickly.
                static auto lastLog = std::chrono::steady_clock::now();
                const auto now = std::chrono::steady_clock::now();
                if (now - lastLog > std::chrono::seconds(2)) {
                    double minVal = 0.0, maxVal = 0.0;
                    cv::minMaxLoc(disp32f, &minVal, &maxVal);
                    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO,
                                                "Stereo disparity range: min=%.2f max=%.2f (post-clamp)\n",
                                                minVal, maxVal);
                    lastLog = now;
                }
            }
            break;

        default:
            break;
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


/** * @brief Process a stereo frame pair (placeholder for future processing)
 * 
 * @param frame Input/output frame pair
 */
void VisionControls::processFramePair(std::pair<cv::Mat, cv::Mat>& stereoFrame) {
    using namespace std;
    using namespace cv;
    using namespace dnn;

    cv::Mat& frameL  = stereoFrame.first;
    cv::Mat& frameR = stereoFrame.second;

    switch (m_CamSettings.mode) {
        case CamModeNormal:
            /* code */
            // m_VideoCalib.DoCalibration(frameL, frameR);
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
        case CmdSelCameraMode: {
            const char* camModes[] = {"Normal", "Disparity", "Calibration"};
            logger->log(Logger::LOG_LVL_INFO, "Setting camera mode to %d\n", cmd->data.u8);
            m_CamSettings.mode = cmd->data.u8;
            break;
        }

        case CmdClrVideoRec:
            logger->log(Logger::LOG_LVL_INFO, "Clearing video recording buffer\n");
            m_StreamInFrame.reset();
            m_VideoRecorder.clear();
            m_LastIncomingVideoName.clear();
            break;

        case CmdSaveVideo: {
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

        case CmdReadStoredVideos: {
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

        case CmdLoadSelectedVideo: {
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

        case CmdDeleteVideo: {
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

        case CmdCalibrationSetState:
            break;

        case CmdCalibrationWrtParams: {
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
}


void VisionControls::mainProc() {
    Logger* logger = Logger::getLoggerInst();

    // Handles jitter smoothing and transmission in its own thread
    Vision::VideoStreamer streamer(
        *m_TxAdapter,
        [this]() {
            std::lock_guard<std::mutex> lock(m_HostIPMutex);
            return m_HostIP;
        },
        35,
        1);
    streamer.start();
    m_VideoRecorder.setFrameRate(Vision::VideoRecording::FrameRate::_30Fps);

    Devices::StereoCam cam(0, 1);
    cam.start(1920, 1080, 30);
    cv::Mat frameL;
    cv::Mat frameR;
    cv::Mat frameStereo;

    int ret = -1;
    cv::Mat frameSim;
    cv::Ptr<cv::StereoBM> stereoBM = cv::StereoBM::create();
    std::pair<cv::Mat, cv::Mat> stereoFramePair;
    int16_t xAccel, yAccel, zAccel;
    int16_t xGyro, yGyro, zGyro;
    while (m_Running.load()) {
        switch(m_StreamStats.streamInStatus) {
            case StreamCameraPairs:
                ret = cam.read(frameL, frameR, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);
                if (ret != 0) {
                    continue;
                }

                stereoFramePair = std::make_pair(frameL, frameR);
                processFramePair(stereoFramePair);
                streamer.pushFrame(stereoFramePair);
                break;

            case StreamStereoCameraMono:
                ret = cam.read(frameL, frameR, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);
                if (ret != 0) {
                    continue;
                }
                stereoFramePair = std::make_pair(frameL, frameR);
                processStereo(frameStereo, stereoFramePair);
                streamer.pushFrame(frameStereo, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);
                break;

            case StreamSim: {
                // We draw a frame from the recording object
                cv::Mat simFrame = m_VideoRecorder.getNextFrame();
                if (simFrame.empty()) {
                    // No frames available, wait a bit
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                frameSim = simFrame;
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
