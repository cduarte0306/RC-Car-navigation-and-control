#ifndef VIDEORECORDING_HPP
#define VIDEORECORDING_HPP

#include <vector>
#include <cstddef>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <cstdint>

#include "app/VideoFrame.hpp"

namespace Vision {
/**
 * @brief Buffers encoded frames into size-limited segments on a background thread.
 */
class VideoRecording {
public:
    enum class FrameRate : uint8_t {
        _15Fps = 66, // 15 frames per second
        _30Fps = 33, // 30 frames per second
        _60Fps = 16  // 60 frames per second
    };

    /**
     * @brief Construct recorder (not started).
     */
    VideoRecording();

    /**
     * @brief Join background thread on destruction.
     */
    ~VideoRecording();

    /**
     * @brief Start the recording worker (idempotent).
     */
    void start();

    /**
     * @brief Stop and join the recording worker (idempotent).
     */
    void stop();

    /**
     * @brief Clear recorded segments.
     */
    void clear();

    /**
     * @brief Enqueue encoded frame for recording.
     */
    void pushFrame(const VideoFrame& frame);

    /**
     * @brief Set the target frame rate for playback.
     */
    void setFrameRate(FrameRate framerate) {
        frameRate_ = framerate;
    }

    /**
     * @brief Reset playback to the beginning.
     */
    void resetPlayback() {
        std::lock_guard<std::mutex> lock(mutex_);
        m_currentFrame = 0;
    }

    /**
     * @brief Get the next frame to be processed.
     */
    VideoFrame getNextFrame();

    /**
     * @brief Snapshot recorded segments (thread-safe copy).
     */
    std::vector<VideoFrame> segments() const;

    /**
     * @brief Configure maximum segment size before rotation.
     */
    void setMaxSegmentSize(std::size_t maxBytes);

    /**
     * @brief List recorded video files in storage path.
     */
    std::vector<std::string> listRecordedFiles() const;

    int loadFile(const std::string& filename);

    /**
     * @brief Save recorded video to file.
     */
    int saveToFile(const std::string& filename);
    
private:
    typedef struct __attribute__((__packed__)) {
        uint32_t length;
        char* payload;
    } VideoPacket;

    const char* VideoStoragePath = "/data/training-videos/";
    FrameRate frameRate_{FrameRate::_30Fps};  // Default to 30 FPS
    std::size_t m_currentFrame = 0;
    std::size_t m_maxSegmentBytes = 0;
    std::vector<VideoFrame> m_Video;
    mutable std::mutex mutex_;
};
}  // namespace Vision

#endif