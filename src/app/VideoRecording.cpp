#include "app/VideoRecording.hpp"

#include <chrono>
#include <algorithm>

VideoRecording::VideoRecording() = default;


VideoRecording::~VideoRecording() {
	stop();
}


void VideoRecording::start() {
	// Placeholder for future background worker; currently a no-op.
}


void VideoRecording::stop() {
	// No background thread yet, but keep API symmetrical.
}


void VideoRecording::clear() {
	std::lock_guard<std::mutex> lock(mutex_);
	for (VideoFrame& frame : m_Video) {
		frame.reset(0);
	}
	m_Video.clear();
	m_currentFrame = 0;
}


void VideoRecording::pushFrame(const VideoFrame& frame) {
	std::lock_guard<std::mutex> lock(mutex_);
	m_Video.push_back(frame);
	if (m_maxSegmentBytes > 0) {
		// Optional trimming by size if required in the future.
		std::size_t totalBytes = 0;
		for (const auto& f : m_Video) {
			totalBytes += f.size();
		}
		while (totalBytes > m_maxSegmentBytes && !m_Video.empty()) {
			totalBytes -= m_Video.front().size();
			m_Video.erase(m_Video.begin());
			if (m_currentFrame > 0) {
				--m_currentFrame;
			}
		}
	}
}


VideoFrame VideoRecording::getNextFrame() {
	static std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
	
	while(std::chrono::steady_clock::now() - lastTime < std::chrono::milliseconds(static_cast<int>(1000 / static_cast<int>(frameRate_)))) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	std::lock_guard<std::mutex> lock(mutex_);
	
	if (m_Video.empty()) {
		return VideoFrame{};
	}

	if (m_currentFrame >= m_Video.size()) {
		m_currentFrame = 0;
	}

	VideoFrame frame = m_Video[m_currentFrame];
	m_currentFrame = (m_currentFrame + 1) % m_Video.size();
	return frame;
}


std::vector<VideoFrame> VideoRecording::segments() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return m_Video;
}


void VideoRecording::setMaxSegmentSize(std::size_t maxBytes) {
	std::lock_guard<std::mutex> lock(mutex_);
	m_maxSegmentBytes = maxBytes;
}

