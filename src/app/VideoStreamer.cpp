#include "app/VideoStreamer.hpp"
#include <cstring>

#include <thread>
#include "Devices/RegisterMap.hpp"

#ifdef HAVE_OPENCV_CUDAIMGPROC
#include <opencv2/cudaimgproc.hpp>
#endif

#include "utils/logger.hpp"


VideoStreamer::VideoStreamer(Adapter::CommsAdapter::NetworkAdapter& txAdapter,
                            std::function<std::string()> destIpProvider,
                            int jpegQuality,
                            std::size_t bufferCapacity)
      : encodeQuality(jpegQuality),
        m_TxAdapter(txAdapter),
        m_DestIpProvider(std::move(destIpProvider)),
        m_Buffer(bufferCapacity), m_BufferStereo(bufferCapacity) {}


VideoStreamer::~VideoStreamer() {
    stop();
}


void VideoStreamer::start() {
    if (m_Running.exchange(true)) {
        return;  // already running
    }
    m_ThreadMono = std::thread(&VideoStreamer::runMono, this);
    m_ThreadStereo = std::thread(&VideoStreamer::runStereo, this);
}


void VideoStreamer::stop() {
    if (!m_Running.exchange(false)) {
        return;
    }
    if (m_ThreadMono.joinable()) {
        m_ThreadMono.join();
    }
    if (m_ThreadStereo.joinable()) {
        m_ThreadStereo.join();
    }
}


int VideoStreamer::decodePacket(const char* pbuf, size_t len, uint8_t& numSegments, uint8_t& segmentID, uint32_t& totalLength, uint16_t& payloadLen, uint64_t& seqId, std::vector<uint8_t>& payload) {
    if (len < sizeof(FragmentHeader) + sizeof(Metadata)) {
        return -1;
    }
    
    // Placeholder implementation
    struct FragmentPayload*  frameSegment = reinterpret_cast<struct FragmentPayload*>(const_cast<char*>(pbuf));
    numSegments  = frameSegment->metadata.numSegments;
    segmentID    = frameSegment->metadata.segmentID;
    totalLength  = frameSegment->metadata.totalLength;
    payloadLen   = frameSegment->metadata.length;
    seqId        = frameSegment->metadata.sequenceID;

    if (payloadLen > 0) {
        payload.resize(payloadLen);
        std::memcpy(payload.data(), frameSegment->payload, payloadLen);
    } else {
        payload.clear();
    }
    return 0;
}


void VideoStreamer::pushFrame(const cv::Mat& frame) {
    if (!m_Running.load()) return;
    if (frame.empty()) return;
    // Lowest-latency path: avoid deep copies; CircularBuffer overwrites when full.
    m_Buffer.push(frame);
}


void VideoStreamer::pushFrame(const std::pair<cv::Mat, cv::Mat>& framePair) {
    const cv::Mat& frameL = framePair.first;
    const cv::Mat& frameR = framePair.second;
    if (!m_Running.load()) return;
    if (frameL.empty() || frameR.empty()) return;
    m_BufferStereo.push(framePair);
}


void VideoStreamer::runMono() {
    // Wait until allowed to run
    RegisterMap* regMap = RegisterMap::getInstance();
    std::optional<std::string> destIpReg;
    while (destIpReg = regMap->get<std::string>(RegisterMap::RegisterKeys::HostIP), !destIpReg.has_value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "VideoStreamer started. Host IP: %s\n", destIpReg->c_str());

    auto lastTime = std::chrono::steady_clock::now();
    m_DestIp = *destIpReg;

    while (m_Running) {
        if (m_Buffer.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        // For lowest latency, always transmit the newest frame.
        cv::Mat frame = m_Buffer.getHead();
        m_Buffer.pop();

        if (frame.empty()) {
            continue;
        }

        transmitFrame(frame);
        m_FrameID++;
    }
}


void VideoStreamer::runStereo() {
    // Wait until allowed to run
    RegisterMap* regMap = RegisterMap::getInstance();
    std::optional<std::string> destIpReg;
    while (destIpReg = regMap->get<std::string>(RegisterMap::RegisterKeys::HostIP), !destIpReg.has_value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "VideoStreamer started. Host IP: %s\n", destIpReg->c_str());
    m_DestIp = *destIpReg;

    std::pair<cv::Mat, cv::Mat> stereoFrames;

    while (m_Running) {
        if (m_BufferStereo.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        // For lowest latency, always transmit the newest stereo pair.
        stereoFrames = m_BufferStereo.getHead();
        m_BufferStereo.pop();

        if (stereoFrames.first.empty() || stereoFrames.second.empty()) {
            continue;
        }

        prepFrame(stereoFrames);
    }
}


int VideoStreamer::transmitFrame(cv::Mat& frame, int frameType, int frameSide) {
    if (frame.empty()) {
        return -1;
    }

    cv::Mat encodeBgr;
    cv::Mat* encodeFrame = &frame;
    if (frame.channels() == 4) {
        cv::cvtColor(frame, encodeBgr, cv::COLOR_BGRA2BGR);
        encodeFrame = &encodeBgr;
    }

    // Encode JPEG
    std::vector<uint8_t> encoded;
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, encodeQuality };
    cv::imencode(".jpg", *encodeFrame, encoded, params);

    size_t totalSize      = encoded.size();
    size_t bytesRemaining = totalSize;
    size_t offset         = 0;
    uint32_t segmentIndex = 0;

    // Number of segments
    uint8_t numSegments = (totalSize + MaxPayloadSize - 1) / MaxPayloadSize;

    FragmentPayload packet;
    Metadata meta;

    std::string destIp = m_DestIp;
    if (destIp.empty()) {
        return -1;
    }

    while (bytesRemaining > 0) {
        // Fill metadata
        meta.sequenceID  = m_FrameID;
        meta.totalLength = static_cast<uint32_t>(totalSize);
        meta.segmentID   = segmentIndex;
        meta.numSegments = numSegments;

        size_t bytesToSend = std::min<std::size_t>(bytesRemaining, MaxPayloadSize);
        meta.length = static_cast<uint16_t>(bytesToSend);

        // Write metadata
        packet.FragmentHeader.frameType = static_cast<uint8_t>(frameType);
        packet.FragmentHeader.frameSide = static_cast<uint8_t>(frameSide);
        packet.metadata = meta;

        // Copy payload bytes
        std::memcpy(packet.payload, encoded.data() + offset, bytesToSend);

        offset         += bytesToSend;
        bytesRemaining -= bytesToSend;
        segmentIndex++;

        // Lowest-latency: send only the bytes we actually have.
        // This avoids sending/zero-filling ~64KB UDP datagrams for small JPEG segments.
        const std::size_t packetSize = sizeof(FragmentHeader) + sizeof(Metadata) + bytesToSend;
        auto* raw = reinterpret_cast<uint8_t*>(&packet);

        m_TxAdapter.send(destIp, raw, packetSize);
    }
    return 0;
}


int VideoStreamer::prepFrame(const std::pair<cv::Mat, cv::Mat>& framePair) {
    cv::Mat& frameL = const_cast<cv::Mat&>(framePair.first);
    cv::Mat& frameR = const_cast<cv::Mat&>(framePair.second);
    if (frameL.empty() || frameR.empty()) {
        return -1;
    }

    if (!frameL.empty()) {
        transmitFrame(frameL, 1, 0);  // frameType=1 (stereo), frameSide=0 (left)
    }

    if (!frameR.empty()) {
        transmitFrame(frameR, 1, 1);  // frameType=1 (stereo), frameSide=1 (right)
    }

    m_FrameID++;
    return 0;
}
