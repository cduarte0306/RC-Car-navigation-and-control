#ifndef VIDEOFRAME_HPP
#define VIDEOFRAME_HPP

#include <vector>
#include <cstddef>
#include <cstdint>
#include <string>
#include <map>
#include <map>


namespace Vision {
/**
 * @brief Represents a single encoded video fragment that can be appended to and queried.
 */
class VideoFrame {
public:
    /**
     * @brief Default construct an empty segment with ID 0.
     */
    VideoFrame() = default;

    VideoFrame(const VideoFrame& other) noexcept = default;

    ~VideoFrame() = default;

    VideoFrame& operator=(const VideoFrame& other) noexcept = default;

    /**
     * @brief Construct a segment with a specific identifier.
     * @param id Segment identifier used when reassembling.
     */
    explicit VideoFrame(std::size_t id);

    /**
     * @brief Set expected segment count and total length for this frame.
     */
    void setExpected(std::size_t expectedSegments, std::size_t expectedTotalLength) noexcept;

    /**
     * @brief Append raw bytes to the segment payload.
     * @param id Segment identifier used when reassembling.
     * @param src Pointer to data buffer.
     * @param length Number of bytes to append.
     */
    void append(int id, const uint8_t* src, std::size_t length);

    /**
     * @brief Append raw bytes to the segment payload (defaults to segment 0).
     */
    void append(const uint8_t* src, std::size_t length);

    /**
     * @brief Append a byte vector to the segment payload.
     * @param id Segment identifier used when reassembling.
     * @param buffer Byte vector to append.
     */
    void append(int id, const std::vector<uint8_t>& buffer);

    /**
     * @brief Append a byte vector to segment 0.
     */
    void append(const std::vector<uint8_t>& buffer);

    /**
     * @brief Get the segment map.
     */
    std::map<int, std::vector<uint8_t>>& getSegmentMap() noexcept {
        return m_FrameSegMap;
    }

    const std::map<int, std::vector<uint8_t>>& getSegmentMap() const noexcept {
        return m_FrameSegMap;
    }

    /**
     * @brief Get the segment identifier.
     */
    std::size_t id() const noexcept;
    
    std::size_t frameID() const noexcept { return frameID_; }

    /**
     * @brief Set the segment identifier.
     */
    void setFrameID(std::size_t id) noexcept { frameID_ = id; }

    /**
     * @brief Access the underlying byte buffer.
     */
    const std::vector<uint8_t> bytes() const noexcept;

    /**
     * @brief Number of segments currently stored.
     */
    std::size_t numSegments() const noexcept { return m_FrameSegMap.size(); }

    /**
     * @brief Expected number of segments.
     */
    std::size_t expectedSegments() const noexcept { return expectedSegments_; }

    /**
     * @brief Expected total length in bytes.
     */
    std::size_t expectedTotalLength() const noexcept { return expectedTotalLength_; }

    /**
     * @brief Current payload size in bytes.
     */
    std::size_t size() const noexcept;

    /**
     * @brief Reset payload and optionally set a new identifier.
     */
    void reset();

private:
    std::size_t frameID_{0};
    std::size_t expectedSegments_{0};
    std::size_t expectedTotalLength_{0};
    std::map<int, std::vector<uint8_t>> m_FrameSegMap;
};
}  // namespace Vision

#endif