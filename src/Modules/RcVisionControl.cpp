#include "RcVisionControl.hpp"
#include "RcMessageLib.hpp"

#include "utils/logger.hpp"
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <sstream>
#include <functional>
#include <chrono>
#include <algorithm>


namespace Modules {
VisionControls::VisionControls(int moduleID, std::string name) : 
    Base(moduleID, name), Adapter::CameraAdapter(name), m_FrameRecvBuff(100) {
    Logger* logger = Logger::getLoggerInst();
    this->m_UdpSocket    = std::make_unique<Network::UdpServer>(io_context, "wlP1p1s0", "enP8p1s0", 5000, 65507);
    this->m_UdpSimSocket = std::make_unique<Network::UdpServer>(io_context, "enP8p1s0", "enP8p1s0", 5000, 65507);
    logger->log(Logger::LOG_LVL_INFO, "Vision object initialized\r\n");

    setPeriod(1000);  // Set the timer thread to service ever second
    this->m_UdpSimSocket->startReceive(std::bind(&VisionControls::recvFrame, this, std::placeholders::_1, std::placeholders::_2), false);
}


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

    frame = cv::imdecode(jpegFrame, cv::IMREAD_COLOR);
}


/**
 * @brief Handles the packetizing and transmission of frame ovfer UDP
 * 
 * @param frame Frame to transmit
 * @return int Error code
 */
int VisionControls::transmitFrames(cv::Mat& frame) {
    if (frame.empty()) {
        return -1;
    }

    // Encode JPEG
    std::vector<uint8_t> encoded;
    encoded.clear();
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 35 };
    cv::imencode(".jpg", frame, encoded, params);

    size_t totalSize      = encoded.size();
    size_t bytesRemaining = totalSize;
    size_t offset         = 0;
    uint32_t segmentIndex = 0;

    // Number of segments
    uint8_t numSegments =
        (totalSize + MaxPayloadSize - 1) / MaxPayloadSize;

    FragmentPayload packet;
    Metadata meta;

    while (bytesRemaining > 0)
    {
        // Fill metadata
        meta.sequenceID  = m_FrameID;
        meta.totalLength = totalSize;
        meta.segmentID   = segmentIndex;
        meta.numSegments = numSegments;

        size_t bytesToSend = std::min((int64_t)(bytesRemaining), (int64_t)MaxPayloadSize);
        meta.length = bytesToSend;

        // Write metadata
        packet.metadata = meta;

        // Copy payload bytes
        std::memcpy(packet.payload,
                    encoded.data() + offset,
                    bytesToSend);

        // (Optional but wise) zero-fill remainder
        if (bytesToSend < MaxPayloadSize) {
            std::memset(packet.payload + bytesToSend,
                        0,
                        MaxPayloadSize - bytesToSend);
        }

        offset         += bytesToSend;
        bytesRemaining -= bytesToSend;
        segmentIndex++;

        // Python version sends FULL fixed-size packet every time
        size_t packetSize =
            sizeof(Metadata) + MaxPayloadSize;

        uint8_t* raw = reinterpret_cast<uint8_t*>(&packet);

        this->m_UdpSocket->transmit(raw, packetSize, m_HostIP);
    }

    m_FrameID++;

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
    
    // If we've not received frames in over 2 seconds, switchback to reading fromthe camera
    if (m_StreamStats.streamInCounter > 2) {
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
    // Main processing loop for the motor controller
    Logger* logger = Logger::getLoggerInst();
    Msg::CircularBuffer<cv::Mat> circBuff(100);

    Modules::RcThread consumer(
        [&]() {
            cv::Mat frame;

            while (!m_StreamerCanRun.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            logger->log(Logger::LOG_LVL_INFO, "Starting video streaming\n");
            long long FrameLength = 0;
            std::vector<uchar> encoded;
            auto lastTime = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();

            while(m_ThreadCanRun) {
                // Reset the receiving frame flag after 5 second
                now = std::chrono::steady_clock::now();

                auto elapsedMs = std::chrono::duration_cast<std::chrono::seconds>(now - lastTime);
                if (elapsedMs.count() > 5) {
                    m_ReceivingFrame = false;
                    lastTime = std::chrono::steady_clock::now();
                }
                
                if (!circBuff.isEmpty()) {
                    frame = circBuff.getHead();
                    if (frame.empty()) {
                        logger->log(Logger::LOG_LVL_WARN, "Received frame is empty\n");
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                        continue;
                    }
                    FrameLength = frame.total() * frame.elemSize();

                    // Feed to the socket
                    transmitFrames(frame);

                    // Advance the circular buffer
                    circBuff.pop();
                }
            }
        }
    );

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

        if (m_StreamerCanRun.load()) {
            circBuff.push(frame);
        }
        
        // streamAdapter->moduleCommand(data, size);
        // this->streamAdapter->(frame.data(), frame.size());
    }
}
}