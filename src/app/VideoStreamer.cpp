#include "app/VideoStreamer.hpp"
#include <cstring>

#include <thread>

#ifdef HAVE_OPENCV_CUDAIMGPROC
#include <opencv2/cudaimgproc.hpp>
#endif

VideoStreamer::VideoStreamer(std::atomic<bool>& canRunFlag,
                                                         Adapter::CommsAdapter::NetworkAdapter& txAdapter,
                                                         std::function<std::string()> destIpProvider,
                                                         int jpegQuality,
                                                         std::size_t bufferCapacity)
        : encodeQuality(jpegQuality),
            m_TxAdapter(txAdapter),
            m_DestIpProvider(std::move(destIpProvider)),
            m_CanRun(canRunFlag),
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


void VideoStreamer::pushFrame(const cv::Mat& frame) {
    if (!m_Running.load()) return;
    if (!m_CanRun.load()) return;
    if (frame.empty()) return;
    m_Buffer.push(frame.clone());
}


void VideoStreamer::pushFrame(const std::pair<cv::Mat, cv::Mat>& framePair) {
    const cv::Mat& frameL = framePair.first;
    const cv::Mat& frameR = framePair.second;
    if (!m_Running.load()) return;
    if (!m_CanRun.load()) return;
    if (frameL.empty()) return;
    m_BufferStereo.push(framePair);
}


void VideoStreamer::runMono() {
    // Wait until allowed to run
    while (m_Running && !m_CanRun.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto lastTime = std::chrono::steady_clock::now();

    auto enforceFps = [&](int targetFps) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        int frameDuration = 1000 / targetFps;
        if (elapsed < frameDuration) {
            std::this_thread::sleep_for(std::chrono::milliseconds(frameDuration - elapsed));
            lastTime = std::chrono::steady_clock::now();
        } else {
            lastTime = now;
        }
    };

    while (m_Running) {
        if (m_Buffer.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        // Copy frame before popping to avoid holding buffer refs
        cv::Mat frame = m_Buffer.getHead().clone();
        m_Buffer.pop();

        // Maintain approximately 30 FPS
        enforceFps(30);

        if (frame.empty()) {
            continue;
        }

        transmitFrame(frame);
        m_FrameID++;
    }
}


void VideoStreamer::runStereo() {
    // Wait until allowed to run
    while (m_Running && !m_CanRun.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto lastTime = std::chrono::steady_clock::now();

    auto enforceFps = [&](int targetFps) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        int frameDuration = 1000 / targetFps;
        if (elapsed < frameDuration) {
            std::this_thread::sleep_for(std::chrono::milliseconds(frameDuration - elapsed));
            lastTime = std::chrono::steady_clock::now();
        } else {
            lastTime = now;
        }
    };

    std::pair<cv::Mat, cv::Mat> stereoFrames;

    while (m_Running) {
        if (m_BufferStereo.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        // Copy frame before popping to avoid holding buffer refs
        stereoFrames = m_BufferStereo.getHead();
        m_BufferStereo.pop();

        // Maintain approximately 30 FPS
        enforceFps(30);

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

    std::string destIp = m_DestIpProvider ? m_DestIpProvider() : std::string();
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

        // Zero-fill remainder (keeps packet size fixed)
        if (bytesToSend < MaxPayloadSize) {
            std::memset(packet.payload + bytesToSend, 0, MaxPayloadSize - bytesToSend);
        }

        offset         += bytesToSend;
        bytesRemaining -= bytesToSend;
        segmentIndex++;

        size_t packetSize = sizeof(Metadata) + MaxPayloadSize;
        uint8_t* raw      = reinterpret_cast<uint8_t*>(&packet);

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
