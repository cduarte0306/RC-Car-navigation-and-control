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

    /**
     * @brief Enqueue a stereo frame pair for transmission (frames are cloned).
     */
    void pushFrame(const std::pair<cv::Mat, cv::Mat>& framePair);

    /**
     * @brief Enqueue a stereo frame for transmission (frame is cloned).
     * 
     * @param frame Input superimposed stereo frame
     * @param xGyro Gyro X-axis
     * @param yGyro Gyro Y-axis
     * @param zGyro Gyro Z-axis
     * @param xAccel Accel X-axis
     * @param yAccel Accel Y-axis
     * @param zAccel Accel Z-axis
     */
    void pushFrame(const cv::Mat& frame, int16_t xGyro, int16_t yGyro, int16_t zGyro, int16_t xAccel, int16_t yAccel, int16_t zAccel);

    /**
     * @brief Decode a received packet into its components.
     * 
     * @param pbuf Pointer to the packet buffer.
     * @param len Length of the packet buffer.
     * @param numSegments Output number of segments for the frame.
     * @param segmentID Output segment ID of this packet.
     * @param totalLength Output total length of the frame.
     * @param payloadLen Output length of the payload in this packet.
     * @param seqId Output sequence ID of the frame.
     * @param payload Output vector to hold the payload data.
     * @return int 0 on success, negative on failure.
     */
    static int decodePacket(const char* pbuf, size_t len, uint8_t& numSegments, uint8_t& segmentID, uint32_t& totalLength, uint16_t& payloadLen, uint64_t& seqId, std::vector<uint8_t>& payload);

private:
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
        uint8_t  frameType;  // 0 = mono, 1 = stereo, 2 = stereo-mono
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

    typedef struct __attribute__((__packed__)) {
        int16_t gx;  // Gyro X-axis
        int16_t gy;  // Gyro Y-axis
        int16_t gz;  // Gyro Z-axis
        int16_t ax;  // Accel X-axis
        int16_t ay;  // Accel Y-axis
        int16_t az;  // Accel Z-axis
    } stereoHeader_t;

    #pragma pack(pop)

    typedef struct {
        stereoHeader_t stereoHeader;
        cv::Mat stereoFrame;
    } stereoPayload;

    static_assert(sizeof(FragmentPayload) <= MaxUDPLen, "FragmentPayload exceeds UDP max payload");

    void runMono();
    void runStereo();
    void runStereoMono();

    /**
     * @brief Transmit a single frame.
     * 
     * @param frame Reference to the frame to transmit
     * @return int 
     */
    int transmitFrame(cv::Mat& frame, int frameType=0, int frameSide=0);

    /**
     * @brief Transmit a stereo frame.
     * 
     * @param stereoFrame Reference to the stereo frame to transmit
     * @return int 
     */
    int transmitFrame(stereoPayload& stereoFrame, int frameType=2);

    /**
     * @brief Transmit a stereo frame pair.
     * 
     * @param framePair Reference to the stereo frame pair to transmit
     * @return int 
     */
    int prepFrame(const std::pair<cv::Mat, cv::Mat>& framePair);

    int encodeQuality;
    Adapter::CommsAdapter::NetworkAdapter& m_TxAdapter;
    std::function<std::string()> m_DestIpProvider;
    std::string m_DestIp;

    uint32_t m_FrameID = 0;

        /**
     * @brief Buffer for mono frames (Usually simulation frames)
     * 
     */
    Msg::CircularBuffer<cv::Mat> m_Buffer;

    /**
     * @brief Buffer for combined stereo frames
     * 
     */
    Msg::CircularBuffer<stereoPayload> m_BufferStereoMono;

    /**
     * @brief Buffer for stereo frame pairs
     * 
     */
    Msg::CircularBuffer<std::pair<cv::Mat, cv::Mat>> m_BufferStereo;
    std::atomic<bool> m_Running{false};
    std::thread m_ThreadMono;
    std::thread m_ThreadStereo;
    std::thread m_ThreadStereoMono;
};
