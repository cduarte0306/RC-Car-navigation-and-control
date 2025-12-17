#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <opencv2/opencv.hpp>

#include "Modules/RcMessageLib.hpp"

/**
 * @brief Handles jitter-smoothing and transmission of video frames on a worker thread.
 */
class VideoStreamer {
public:
    using TxFn = std::function<int(cv::Mat&)>;

    /**
     * @brief Construct a streamer.
     * @param txFn callable used to transmit a frame.
     * @param canRunFlag external flag that gates streaming readiness.
     * @param bufferCapacity number of frames buffered for jitter smoothing.
     */
    VideoStreamer(TxFn txFn, std::atomic<bool>& canRunFlag, std::size_t bufferCapacity = 100);


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

    TxFn m_TxFn;
    std::atomic<bool>& m_CanRun;
    Msg::CircularBuffer<cv::Mat> m_Buffer;
    std::atomic<bool> m_Running{false};
    std::thread m_Thread;
};
