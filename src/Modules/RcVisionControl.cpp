#include "RcVisionControl.hpp"
#include "RcMessageLib.hpp"
#include "lib/VideoStreamer.hpp"

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
#include <opencv2/cudaimgproc.hpp>
#include <gst/gst.h>
#include <sstream>
#include <functional>
#include <chrono>
#include <algorithm>

#define STREAM_PORT 5000

#define MODEL_PATH  "/opt/rc-car/models/model-small.onnx"


namespace Modules {
VisionControls::VisionControls(int moduleID, std::string name) : 
    Base(moduleID, name), Adapter::CameraAdapter(name), m_FrameRecvBuff(100) {
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
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create vision network adapters\r\n");
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

    m_DnnNet = cv::dnn::readNet(MODEL_PATH);
    if (m_DnnNet.empty()) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to load DNN model at %s\n", MODEL_PATH);
    } else {
        if (useCuda) {
            m_DnnNet.setPreferableBackend(DNN_BACKEND_CUDA);
            m_DnnNet.setPreferableTarget(DNN_TARGET_CUDA);
            logger->log(Logger::LOG_LVL_INFO, "DNN model loaded with CUDA backend\n");
        } else {
            m_DnnNet.setPreferableBackend(DNN_BACKEND_OPENCV);
            m_DnnNet.setPreferableTarget(DNN_TARGET_CPU);
            logger->log(Logger::LOG_LVL_INFO, "DNN model loaded with CPU backend\n");
        }
    }

    // Start receiving frames
    this->CommsAdapter->startReceive(
        *m_RxAdapter,
        std::bind(&VisionControls::recvFrame, this, std::placeholders::_1, std::placeholders::_2),
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
    if (m_Writer.isOpened()) {
        m_Writer.release();
    }

    Logger* logger = Logger::getLoggerInst();
    std::lock_guard<std::mutex> lock(m_HostIPMutex); 
    m_HostIP = host;
    m_StreamerCanRun.store(true);
    logger->log(Logger::LOG_LVL_INFO, "Configured host: %s\n", host.c_str());
    return 0;
}


/**
 * @brief Received frame callback
 * 
 * @param pbuf Pointer to UDP recive  buffer
 * @param length Length of data received
 */
void VisionControls::recvFrame(const uint8_t* pbuf, size_t length) {
    if (!pbuf) return;

    static bool doOnce = false;

    Logger* logger = Logger::getLoggerInst();
    FramePackets frameFragment;
    std::vector<uint8_t> segmentData;
    static uint64_t frameID = 0;

    // Decode the image
    struct FragmentPayload* frameSegment = reinterpret_cast<struct FragmentPayload*>(const_cast<uint8_t*>(pbuf));
    uint8_t numSegments = frameSegment->metadata.numSegments;

    if (m_SegmentMap.segmentMap.size() > 0 && (m_SegmentMap.frameID != frameSegment->metadata.sequenceID)) {
        // Clear the map
        m_SegmentMap.segmentMap.clear();
    }

    m_SegmentMap.frameID = frameSegment->metadata.sequenceID;
    m_SegmentMap.numSegments = frameSegment->metadata.numSegments;

    // Append frame as long as the index is the same
    segmentData.resize(frameSegment->metadata.length);

    // Copy the data into the buffer
    std::memcpy(segmentData.data(), frameSegment->payload, frameSegment->metadata.length);

    // Store the frame ID
    frameID = frameSegment->metadata.sequenceID;
    m_SegmentMap.segmentMap[frameSegment->metadata.segmentID] = std::move(segmentData);

    if (m_SegmentMap.segmentMap.size() == numSegments) {
        // Validate that the assembled size matches the advertised total length
        size_t totalReceived = 0;
        for (const auto& kv : m_SegmentMap.segmentMap) {
            totalReceived += kv.second.size();
        }

        if (totalReceived != frameSegment->metadata.totalLength) {
            logger->log(Logger::LOG_LVL_WARN,
                        "Frame %llu size mismatch (expected %u, got %zu)\n",
                        static_cast<unsigned long long>(m_SegmentMap.frameID),
                        frameSegment->metadata.totalLength,
                        totalReceived);
            m_SegmentMap.segmentMap.clear();
            return;  // drop corrupt frame
        }

        // Push the segment map
        m_FrameRecvBuff.push(m_SegmentMap);
        m_SegmentMap.segmentMap.clear();

        if (m_StreamStats.streamInStatus == VisionControls::StreamInOff) {
            doOnce = false;
        }

        m_StreamStats.streamInStatus = VisionControls::StreamInOn;
        m_StreamStats.streamInCounter = 0;  // Reset the counter

        if (!doOnce) {
            logger->log(Logger::LOG_LVL_INFO, "Stream in started...\n");
            doOnce = true;
        }
    }
}


/**
 * @brief Decode a JPEG frame from the received segments
 * 
 * @param frame Output frame
 * @param frameEntry FramePackets entry containing segments
 */
void VisionControls::decodeJPEG(cv::Mat& frame, FramePackets& frameEntry) {
    auto& frameMap = frameEntry.segmentMap;
    const uint8_t numSegments = frameEntry.numSegments;

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
 * @brief Process a frame (placeholder for future processing)
 * 
 * @param frame Input/output frame
 */
void VisionControls::processFrame(cv::Mat& frame) {
    using namespace std;
    using namespace cv;
    using namespace dnn;

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

    switch (m_CamSettings.mode)
    {
    case CamModeNormal:
        /* code */
        break;
    
    case CamModeDepth: {
        static bool logMissingModel = false;
        if (m_DnnNet.empty()) {
            if (!logMissingModel) {
                Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "CamModeDepth selected but DNN model is not loaded\n");
                logMissingModel = true;
            }
            break;
        }

        if (frame.empty()) {
            break;
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

        m_DnnNet.setInput(blob);

        cv::Mat output;
        try {
            output = m_DnnNet.forward(getOutputsNames(m_DnnNet)[0]);
        } catch (const cv::Exception& e) {
            Logger::getLoggerInst()->log(
                Logger::LOG_LVL_ERROR,
                "DNN forward failed: %s (blob shape: NCHW=%d,%d,%d,%d)\n",
                e.what(),
                blob.size[0], blob.size[1], blob.size[2], blob.size[3]);
            break;
        }

        if (output.empty()) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "DNN forward returned empty output\n");
            break;
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
            break;
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
int VisionControls::moduleCommand_(char* pbuf, size_t len) {
    // Currently no commands implemented
    CameraCommand* cmd = reinterpret_cast<CameraCommand*>(pbuf);
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

        case CmdSelMode:
            logger->log(Logger::LOG_LVL_INFO, "Setting camera mode to %d\n", cmd->data.u8);
            m_CamSettings.mode = cmd->data.u8;
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
    static bool doOnce = false;
    Logger* logger = Logger::getLoggerInst();

    if (m_StreamStats.streamInStatus == VisionControls::StreamInOn) {
       doOnce = false; 
    }
    
    // If we've not received frames in over 1 second, switchback to reading fromthe camera
    if (m_StreamStats.streamInCounter > 1) {
        if (!doOnce) {
            logger->log(Logger::LOG_LVL_INFO, "Timeout detected. Stream in is OFF\n");
            doOnce = true;
        }
        m_StreamStats.streamInStatus = VisionControls::StreamInOff;
    } else {
        m_StreamStats.streamInCounter ++;
    }
}


void VisionControls::mainProc() {
    Logger* logger = Logger::getLoggerInst();

    // Handles jitter smoothing and transmission in its own thread
    VideoStreamer streamer(
        m_StreamerCanRun,
        *m_TxAdapter,
        [this]() {
            std::lock_guard<std::mutex> lock(m_HostIPMutex);
            return m_HostIP;
        },
        35,
        100);
    streamer.start();

    cv::VideoCapture cap(
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1,format=NV12 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink",
        cv::CAP_GSTREAMER
    );

    if (!cap.isOpened()) {
        logger->log(Logger::LOG_LVL_ERROR, "ERROR: Could not open /dev/video0\n");
        return;
    }

    logger->log(Logger::LOG_LVL_INFO, "Opened camera at node: /dev/video0\n");
    cv::Mat frame;
    int size;
    while (true) {
        switch(m_StreamStats.streamInStatus) {
            case StreamInOff:
            cap.read(frame);
            break;

            case StreamInOn: {
                if (m_FrameRecvBuff.isEmpty())
                    continue;
                FramePackets frameEntry = m_FrameRecvBuff.getHead();
                decodeJPEG(frame, frameEntry);
                if (frame.empty()) {
                    logger->log(Logger::LOG_LVL_WARN, "Could not decode JPEG\n");
                    m_FrameRecvBuff.pop();
                    continue;
                }

                m_FrameRecvBuff.pop();
                break;
            }
        }

        if (frame.empty()) {
            continue;
        }

        processFrame(frame);

        if (m_StreamerCanRun.load()) {
            streamer.pushFrame(frame);
        }
    }
}
}