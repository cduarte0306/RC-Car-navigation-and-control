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
     * @param txAdapter outbound network adapter used to send packets.
     * @param destIpProvider callable returning the current destination IP (thread-safe in caller).
     * @param jpegQuality JPEG quality [0-100]; defaults to 35.
     * @param bufferCapacity number of frames buffered for jitter smoothing.
     */
    VideoStreamer(Adapter::CommsAdapter::NetworkAdapter& txAdapter,
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

private:
    void run();

    int encodeQuality;
    Adapter::CommsAdapter::NetworkAdapter& m_TxAdapter;
    std::function<std::string()> m_DestIpProvider;
    std::string m_DestIp;
    Msg::CircularBuffer<cv::Mat> m_Buffer;
    std::atomic<bool> m_Running{false};
    std::thread m_Thread;

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
    #pragma pack(pop)

    static constexpr std::size_t MaxPayloadSize = MaxUDPLen - sizeof(Metadata);

    #pragma pack(push, 1)
    struct FragmentPayload {
        struct Metadata metadata;
        uint8_t payload[MaxPayloadSize];
    };
    #pragma pack(pop)

    static_assert(sizeof(FragmentPayload) <= MaxUDPLen, "FragmentPayload exceeds UDP max payload");

    uint32_t m_FrameID = 0;

    int transmitFrame(cv::Mat& frame);
};
