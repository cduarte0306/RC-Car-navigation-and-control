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
            m_Buffer(bufferCapacity) {}


VideoStreamer::~VideoStreamer() {
    stop();
}


void VideoStreamer::start() {
    if (m_Running.exchange(true)) {
        return;  // already running
    }
    m_Thread = std::thread(&VideoStreamer::run, this);
}


void VideoStreamer::stop() {
    if (!m_Running.exchange(false)) {
        return;
    }
    if (m_Thread.joinable()) {
        m_Thread.join();
    }
}


void VideoStreamer::pushFrame(const cv::Mat& frame) {
    if (!m_Running.load()) return;
    if (!m_CanRun.load()) return;
    if (frame.empty()) return;
    m_Buffer.push(frame.clone());
}


void VideoStreamer::run() {
    // Wait until allowed to run
    while (m_Running && !m_CanRun.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto lastTime = std::chrono::steady_clock::now();

    while (m_Running) {
        if (m_Buffer.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        // Copy frame before popping to avoid holding buffer refs
        cv::Mat frame = m_Buffer.getHead().clone();
        m_Buffer.pop();

        // Maintain approximately 30 FPS
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        if (elapsed < 33) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33 - elapsed));
            lastTime = std::chrono::steady_clock::now();
        } else {
            lastTime = now;
        }

        if (frame.empty()) {
            continue;
        }

        transmitFrame(frame);
    }
}


int VideoStreamer::transmitFrame(cv::Mat& frame) {
    if (frame.empty()) {
        return -1;
    }

#ifdef HAVE_OPENCV_CUDAIMGPROC
    // Optional GPU pass-through; extend with filters if needed
    cv::cuda::GpuMat gsrc;
    gsrc.upload(frame);
    gsrc.download(frame);
#endif

    // Encode JPEG
    std::vector<uint8_t> encoded;
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, encodeQuality };
    cv::imencode(".jpg", frame, encoded, params);

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

    m_FrameID++;
    return 0;
}
