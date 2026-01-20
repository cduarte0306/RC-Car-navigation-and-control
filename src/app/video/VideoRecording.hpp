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
#include <unordered_map>

#include "app/video/VideoFrame.hpp"

#include <opencv2/opencv.hpp>


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
    void pushFrame(const cv::Mat& frame);

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
    cv::Mat getNextFrame();

    /**
     * @brief Snapshot recorded segments (thread-safe copy).
     */
    std::vector<cv::Mat> segments() const;

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

    /**
     * @brief Get the video storage path
     */
    const char* getStoragePath() const {
        return VideoStoragePath;
    }

    /**
     * @brief Configure loaded video path
     */
    int configLoadedVideo();

    /**
     * @brief Get the name of the loaded video file
     * 
     */
    std::string getLoadedVideoName();

    /**
     * @brief Delete a video file from storage
     * 
     * @param filename Name of the video file to delete
     * @return int 0 on success, -1 on failure
     */
    int deleteVideo(const std::string& filename);
    
private:
    typedef struct __attribute__((__packed__)) {
        uint8_t segId;
        uint32_t segLength;
        char* payload;
    } segmentData_t;
    typedef struct __attribute__((__packed__)) {
        struct __attribute__((__packed__)) {
            uint32_t length;
            uint8_t numSegments;
        } header;
        segmentData_t segments[];
    } VideoPacket;

    const char* VideoStoragePath = "/data/training-videos/";
    FrameRate frameRate_{FrameRate::_30Fps};  // Default to 30 FPS
    std::size_t m_currentFrame = 0;
    std::size_t m_maxSegmentBytes = 0;
    // std::vector<VideoFrame> m_Video;
    std::vector<cv::Mat> m_Video;;
    std::vector<std::string> storedVideoCache;
    std::string m_VideoPath;

    mutable std::mutex mutex_;
};
}  // namespace Vision

#endif