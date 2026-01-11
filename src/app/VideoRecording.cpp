#include "app/VideoRecording.hpp"

#include <chrono>
#include <algorithm>
#include <string>
#include <cstring>
#include <filesystem>
#include "utils/logger.hpp"


namespace Vision {
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
		frame.reset();
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


std::vector<std::string> VideoRecording::listRecordedFiles() const {
	std::vector<std::string> fileList;
	if (!std::filesystem::exists(VideoStoragePath)) {
		return fileList; // Return empty if path doesn't exist
	}

	for (const auto& entry : std::filesystem::directory_iterator(VideoStoragePath)) {
		if (entry.is_regular_file()) {
			if (entry.path().extension() != ".rcv")
				continue;
			fileList.push_back(entry.path().filename().string());
		}
	}
	return fileList;
}


int VideoRecording::loadFile(const std::string& filename) {
	std::lock_guard<std::mutex> lock(mutex_);
	m_Video.clear();
	m_currentFrame = 0;

	std::string fullPath = std::string(VideoStoragePath) + filename;
	FILE* file = fopen(fullPath.c_str(), "rb");
	if (!file) {
		Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to open video file for reading: %s\n", fullPath.c_str());
		return -1;
	}

	const size_t bufferSize = 1024 * 1024; // 1 MB buffer
	std::vector<uint8_t> buffer(bufferSize);
	size_t bytesRead = 0;

	while ((bytesRead = fread(buffer.data(), 1, bufferSize, file)) > 0) {
		VideoFrame frame;
		frame.append(buffer.data(), bytesRead);
		m_Video.push_back(frame);
	}

	fclose(file);
	Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Loaded video recording from file: %s\n", fullPath.c_str());
	return 0; // Success
}


int VideoRecording::saveToFile(const std::string& filename) {
	std::lock_guard<std::mutex> lock(mutex_);
	if (m_Video.empty()) {
		Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Recording is empty.\n");
		return -1; // No video to save
	}

	if (!std::filesystem::exists(VideoStoragePath)) {
		std::filesystem::create_directories(VideoStoragePath);
	}

	// For now, only handle MOV type files. We replace the extension and keep the rest of the file name
	std::string fileName_ = filename;
	if (fileName_.find("MOV") != std::string::npos) {
		fileName_.erase(fileName_.find("MOV"), fileName_.length());
		fileName_ += "rcv";
	}

	std::string fullPath = std::string(VideoStoragePath) + fileName_;
	FILE* file = fopen(fullPath.c_str(), "wb");
	if (!file) {
		Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to open file for writing: %s\n", fullPath.c_str());
		return -1;
	}

	// Write each frame's bytes to the file
	for (const VideoFrame& frame : m_Video) {
		VideoRecording::VideoPacket packet;
		packet.length = frame.size();
		std::unique_ptr<uint8_t[]> bufferOut(new uint8_t[packet.length + sizeof(packet.length)]);
		std::memcpy(bufferOut.get(), &packet.length, sizeof(packet.length));
		std::memcpy(bufferOut.get() + sizeof(packet.length), frame.bytes().data(), packet.length);
		if (!frame.bytes().empty()) {
			size_t written = fwrite(bufferOut.get(), 1, packet.length + sizeof(packet.length), file);
			if (written != frame.bytes().size() + sizeof(packet.length)) {
				// Handle write error if needed
				Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Error writing video frame to file: %s\n", fullPath.c_str());
				fclose(file);
				return -1;
			}
		}
	}
	Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Video recording saved to file: %s\n", fullPath.c_str());
	fclose(file);	
	return 0; // Success
}

}  // namespace Vision
