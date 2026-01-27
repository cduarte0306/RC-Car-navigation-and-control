#include "app/video/VideoStreamer.hpp"
#include <cstring>
#include <algorithm>

#include <thread>
#include "Devices/RegisterMap.hpp"

#ifdef HAVE_OPENCV_CUDAIMGPROC
#include <opencv2/cudaimgproc.hpp>
#endif

#include "utils/logger.hpp"


namespace Vision {
VideoStreamer::VideoStreamer(Adapter::CommsAdapter::NetworkAdapter& txAdapter,
                            std::size_t bufferCapacity)
      : m_TxAdapter(txAdapter),
        m_Buffer(bufferCapacity), 
        m_BufferStereo(bufferCapacity),
        m_BufferStereoMono(bufferCapacity) {}


VideoStreamer::~VideoStreamer() {
    stop();
}


void VideoStreamer::start() {
    if (m_Running.exchange(true)) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "VideoStreamer already running.\n");
        return;  // already running
    }

    m_ThreadMono = std::thread(&VideoStreamer::runMono, this);
    m_ThreadStereo = std::thread(&VideoStreamer::runStereo, this);
    m_ThreadStereoMono = std::thread(&VideoStreamer::runStereoMono, this);

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "VideoStreamer started.\n");
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
    if (m_ThreadStereoMono.joinable()) {
        m_ThreadStereoMono.join();
    }

    m_Buffer.flush();
    m_BufferStereoMono.flush();
    m_BufferStereo.flush();

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "VideoStreamer stopped.\n");
}


int VideoStreamer::setJpegQuality(int quality) {
    if (quality < 1 || quality > 100) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "JPEG quality must be between 1 and 100.\n");
        return -1;
    }
    encodeQuality = quality;
    return 0;
}


int VideoStreamer::setStreamFrameRate(FrameRate fps) {
    const uint8_t rawFps = static_cast<uint8_t>(fps);

    switch (fps) {
        case FrameRate::_5Fps:
            frameIntervalMs.store(250);
            break;

        case FrameRate::_10Fps:
            frameIntervalMs.store(100);
            break;
        
        case FrameRate::_15Fps:
            frameIntervalMs.store(66);
            break;
        case FrameRate::_30Fps:
            frameIntervalMs.store(33);
            break;
        case FrameRate::_60Fps:
            frameIntervalMs.store(16);
            break;
        default:
            // Allow callers to pass a raw FPS value (e.g. 5/15/30) via cast.
            if (rawFps == 0 || rawFps > 120) {
                Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Unsupported frame rate.\n");
                return -1;
            }
            frameIntervalMs.store(static_cast<int>(1000 / rawFps));
            return 0;
    }
    return 0;
}


int VideoStreamer::decodePacket(const char* pbuf, size_t len, VideoPacket& packet) {
    if (len < sizeof(FragmentHeader) + sizeof(Metadata)) {
        return -1;
    }

    FragmentHeader hdr{};
    Metadata meta{};
    std::memcpy(&hdr, pbuf, sizeof(hdr));
    std::memcpy(&meta, pbuf + sizeof(hdr), sizeof(meta));

    const std::size_t payloadLen = meta.length;
    const std::size_t headerLen = sizeof(hdr) + sizeof(meta);
    if (len < headerLen + payloadLen) {
        return -1;
    }

    packet.setNumSegments(meta.numSegments);
    packet.setSegmentID(meta.segmentID);
    packet.setTotalLength(meta.totalLength);
    packet.setLength(static_cast<uint32_t>(payloadLen));
    packet.setPayloadLen(static_cast<uint64_t>(payloadLen));
    packet.setSequenceID(meta.sequenceID);

    const std::size_t nameLen = strnlen(meta.videoName, sizeof(meta.videoName));
    try {
        packet.setVideoName(std::string(meta.videoName, nameLen));
    } catch (const std::invalid_argument&) {
        return -1;
    }

    const uint8_t* payloadPtr = reinterpret_cast<const uint8_t*>(pbuf + headerLen);
    packet.setPayload(std::vector<uint8_t>(payloadPtr, payloadPtr + payloadLen));

    return 0;
}


void VideoStreamer::pushFrame(const cv::Mat& frame,  int16_t xGyro, int16_t yGyro, int16_t zGyro, int16_t xAccel, int16_t yAccel, int16_t zAccel, cv::Matx44d& Q) {
    if (!m_Running.load()) return;
    if (frame.empty()) return;
    // Lowest-latency path: avoid deep copies; CircularBuffer overwrites when full.
    stereoPayload payload{};
    payload.stereoHeader.gx = xGyro;
    payload.stereoHeader.gy = yGyro;
    payload.stereoHeader.gz = zGyro;
    payload.stereoHeader.ax = xAccel;
    payload.stereoHeader.ay = yAccel;
    payload.stereoHeader.az = zAccel;
    for (size_t i = 0; i < QSize; ++i) {
        payload.stereoHeader.Q[i] = Q.val[i];
    }
    payload.Q = Q;
    payload.stereoFrame = frame;
    m_BufferStereoMono.push(payload);
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

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "VideoStreamer (mono) started. Host IP: %s\n", destIpReg->c_str());

    m_DestIp = *destIpReg;

    while (m_Running) {
        if (m_Buffer.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            continue;
        }

        VideoStreamer::throttleFps(frameIntervalMs.load());

        // For lowest latency, always transmit the newest frame.
        cv::Mat frame;
        do {
            frame = m_Buffer.getHead();
            m_Buffer.pop();
        } while (!m_Buffer.isEmpty());

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

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "VideoStreamer (stereo) started. Host IP: %s\n", destIpReg->c_str());
    m_DestIp = *destIpReg;

    std::pair<cv::Mat, cv::Mat> stereoFrames;

    while (m_Running) {
        if (m_BufferStereo.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            continue;
        }

        VideoStreamer::throttleFps(frameIntervalMs.load());

        // For lowest latency, always transmit the newest stereo pair.
        do {
            stereoFrames = m_BufferStereo.getHead();
            m_BufferStereo.pop();
        } while (!m_BufferStereo.isEmpty());

        if (stereoFrames.first.empty() || stereoFrames.second.empty()) {
            continue;
        }

        prepFrame(stereoFrames);
    }
}


void VideoStreamer::runStereoMono() {
    // Wait until allowed to run
    RegisterMap* regMap = RegisterMap::getInstance();
    std::optional<std::string> destIpReg;
    while (destIpReg = regMap->get<std::string>(RegisterMap::RegisterKeys::HostIP), !destIpReg.has_value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "VideoStreamer (stereo-mono) started. Host IP: %s\n", destIpReg->c_str());
    m_DestIp = *destIpReg;

    stereoPayload stereoFrame;

    while (m_Running) {
        if (m_BufferStereoMono.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            continue;
        }

        // Point cloud transmission is hard coded to 5 frames per second deu to bandwith limitations
        VideoStreamer::throttleFps(static_cast<int>(VideoStreamer::FrameRate::_5Fps));

        // For lowest latency, always transmit the newest frame.
        do {
            stereoFrame = m_BufferStereoMono.getHead();
            m_BufferStereoMono.pop();
        } while (!m_BufferStereoMono.isEmpty());

        if (stereoFrame.stereoFrame.empty()) {
            continue;
        }

        // transmitFrame(stereoFrame, 2, 0);  // frameType=2 (stereo-mono), frameSide=0
        transmitPointCloud(stereoFrame);
        m_FrameID++;
    }
}


int VideoStreamer::transmitPointCloud(stereoPayload& stereoFrame) {
    cv::Mat& frame = stereoFrame.stereoFrame;
    if (frame.empty()) {
        return -1;
    }

    std::vector<uint8_t> dataOut;
    const uint32_t dataLen = static_cast<uint32_t>(frame.total() * frame.elemSize());
    dataOut.resize(sizeof(stereoHeader_t) + dataLen);

    // Append gyro header at the beginning
    stereoHeader_t* headerPtr = reinterpret_cast<stereoHeader_t*>(dataOut.data());
    headerPtr->gx   = stereoFrame.stereoHeader.gx;
    headerPtr->gy   = stereoFrame.stereoHeader.gy;
    headerPtr->gz   = stereoFrame.stereoHeader.gz;
    headerPtr->ax   = stereoFrame.stereoHeader.ax;
    headerPtr->ay   = stereoFrame.stereoHeader.ay;
    headerPtr->az   = stereoFrame.stereoHeader.az;
    headerPtr->rows = frame.rows;
    headerPtr->cols = frame.cols;
    headerPtr->type = frame.type();
    headerPtr->elemSize = frame.elemSize();
    headerPtr->channels = frame.channels();

    for (int i = 0; i < QSize; i++) {
        headerPtr->Q[i] = stereoFrame.Q.val[i];
    }

    if (dataLen > 0) {
        std::memcpy(dataOut.data() + sizeof(stereoHeader_t), frame.data, dataLen);
    }

    size_t totalSize      = dataOut.size();  // header + JPEG data
    size_t bytesRemaining = totalSize;
    size_t offset         = 0;
    uint32_t segmentIndex = 0;

    // Number of segments
    uint8_t numSegments = (totalSize + MaxPayloadSize - 1) / MaxPayloadSize;

    FragmentPayload packet{};
    Metadata meta{};

    std::string destIp = m_DestIp;
    if (destIp.empty()) {
        return -1;
    }

    while (bytesRemaining > 0) {
        // Fill metadata
        std::memset(meta.videoName, 0, sizeof(meta.videoName));
        meta.sequenceID  = m_FrameID;
        meta.totalLength = static_cast<uint32_t>(totalSize);
        meta.segmentID   = segmentIndex;
        meta.numSegments = numSegments;

        size_t bytesToSend = std::min<std::size_t>(bytesRemaining, MaxPayloadSize);
        meta.length = static_cast<uint16_t>(bytesToSend);

        // Write metadata
        packet.FragmentHeader.frameType = VideoStreamer::DisparityType;
        packet.metadata = meta;

        // Copy payload bytes
        std::memcpy(packet.payload, dataOut.data() + offset, bytesToSend);

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

    FragmentPayload packet{};
    Metadata meta{};

    std::string destIp = m_DestIp;
    if (destIp.empty()) {
        return -1;
    }

    while (bytesRemaining > 0) {
        // Fill metadata
        std::memset(meta.videoName, 0, sizeof(meta.videoName));
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
} // namespace Vision
