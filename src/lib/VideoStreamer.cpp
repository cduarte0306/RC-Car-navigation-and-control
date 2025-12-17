#include "lib/VideoStreamer.hpp"

#include <thread>

VideoStreamer::VideoStreamer(TxFn txFn, std::atomic<bool>& canRunFlag, std::size_t bufferCapacity)
    : m_TxFn(std::move(txFn)), m_CanRun(canRunFlag), m_Buffer(bufferCapacity) {}


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

        if (m_TxFn) {
            m_TxFn(frame);
        }
    }
}
