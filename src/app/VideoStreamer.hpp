#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include <opencv2/opencv.hpp>

#include "Modules/AdapterBase.hpp"
#include "Modules/RcMessageLib.hpp"

/**
 * @brief Handles jitter-smoothing and transmission of video frames on a worker thread.
 */
class VideoStreamer {
public:
    /**
     * @brief Construct a streamer.
     * @param canRunFlag external flag that gates streaming readiness.
     * @param txAdapter outbound network adapter used to send packets.
     * @param destIpProvider callable returning the current destination IP (thread-safe in caller).
     * @param jpegQuality JPEG quality [0-100]; defaults to 35.
     * @param bufferCapacity number of frames buffered for jitter smoothing.
     */
    VideoStreamer(std::atomic<bool>& canRunFlag,
                  Adapter::CommsAdapter::NetworkAdapter& txAdapter,
                  std::function<std::string()> destIpProvider,
                  int jpegQuality = 35,
                  std::size_t bufferCapacity = 100);


    /**
     * @brief Destructor; stops and joins the worker if running.
     */
    ~VideoStreamer();


    // Non-copyable
    VideoStreamer(const VideoStreamer&) = delete;
    VideoStreamer& operator=(const VideoStreamer&) = delete;


    /**
     * @brief Start the streaming thread (idempotent).
     */
    void start();


    /**
     * @brief Stop and join the streaming thread (idempotent).
     */
    void stop();


    /**
     * @brief Enqueue a frame for transmission (frame is cloned).
     */
    void pushFrame(const cv::Mat& frame);

    /**
     * @brief Enqueue a stereo frame pair for transmission (frames are cloned).
     */
    void pushFrame(const std::pair<cv::Mat, cv::Mat>& framePair);

private:
    void runMono();
    void runStereo();

    int encodeQuality;
    Adapter::CommsAdapter::NetworkAdapter& m_TxAdapter;
    std::function<std::string()> m_DestIpProvider;
    std::atomic<bool>& m_CanRun;
    Msg::CircularBuffer<cv::Mat> m_Buffer;
    Msg::CircularBuffer<std::pair<cv::Mat, cv::Mat>> m_BufferStereo;
    std::atomic<bool> m_Running{false};
    std::thread m_ThreadMono;
    std::thread m_ThreadStereo;

    // Packetization helpers (packed to avoid padding inflating packet size)
    static constexpr long long MaxUDPLen = 65507;
    #pragma pack(push, 1)
    struct Metadata {
        uint32_t sequenceID;
        uint8_t  segmentID;
        uint8_t  numSegments;
        uint32_t totalLength;
        uint16_t length;

    };

    struct FragmentHeader {
        uint8_t  frameType;  // 0 = mono, 1 = stereo
        uint8_t  frameSide;  // 0 = left, 1 = right
    };
    #pragma pack(pop)

    static constexpr std::size_t MaxPayloadSize = MaxUDPLen - sizeof(Metadata) - sizeof(FragmentHeader);

    #pragma pack(push, 1)
    struct FragmentPayload {
        struct FragmentHeader FragmentHeader;
        struct Metadata metadata;
        uint8_t payload[MaxPayloadSize];
    };
    #pragma pack(pop)

    static_assert(sizeof(FragmentPayload) <= MaxUDPLen, "FragmentPayload exceeds UDP max payload");

    uint32_t m_FrameID = 0;

    /**
     * @brief Transmit a single frame.
     * 
     * @param frame Reference to the frame to transmit
     * @return int 
     */
    int transmitFrame(cv::Mat& frame, int frameType=0, int frameSide=0);

    /**
     * @brief Transmit a stereo frame pair.
     * 
     * @param framePair Reference to the stereo frame pair to transmit
     * @return int 
     */
    int prepFrame(const std::pair<cv::Mat, cv::Mat>& framePair);
};
