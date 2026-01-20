#include "app/video/VideoRecording.hpp"

#include <chrono>
#include <algorithm>
#include <string>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include "utils/logger.hpp"
#include <nlohmann/json.hpp>


namespace Vision {
VideoRecording::VideoRecording() {
	// Load from filepath
	const std::filesystem::path videoConfigPath = std::filesystem::path(VideoStoragePath) / "video-config.json";
	std::ifstream fileIn(videoConfigPath);
	if (fileIn.is_open()) {
		nlohmann::json videoLoadConfig;
		fileIn >> videoLoadConfig;
		if (videoLoadConfig.contains("path")) {
			m_VideoPath = videoLoadConfig["path"].get<std::string>();;
			loadFile(m_VideoPath);
		}
	}
}


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
	m_Video.clear();
	m_currentFrame = 0;
}


void VideoRecording::pushFrame(const cv::Mat& frame) {
	std::lock_guard<std::mutex> lock(mutex_);
	m_Video.push_back(frame);
	if (m_maxSegmentBytes > 0) {
		// Optional trimming by size if required in the future.
		std::size_t totalBytes = 0;
		for (const auto& f : m_Video) {
			totalBytes += f.total() * f.elemSize();
		}
		while (totalBytes > m_maxSegmentBytes && !m_Video.empty()) {
			totalBytes -= m_Video.front().total() * m_Video.front().elemSize();
			m_Video.erase(m_Video.begin());
			if (m_currentFrame > 0) {
				--m_currentFrame;
			}
		}
	}
}


cv::Mat VideoRecording::getNextFrame() {
	std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
	
	while(std::chrono::steady_clock::now() - lastTime < std::chrono::milliseconds(static_cast<int>(1000 / static_cast<int>(frameRate_)))) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	std::lock_guard<std::mutex> lock(mutex_);
	
	if (m_Video.empty()) {
		return cv::Mat{};
	}

	if (m_currentFrame >= m_Video.size()) {
		m_currentFrame = 0;
	}

	cv::Mat frame = m_Video[m_currentFrame];
	m_currentFrame = (m_currentFrame + 1) % m_Video.size();
	return frame;
}


std::vector<cv::Mat> VideoRecording::segments() const {
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


int VideoRecording::configLoadedVideo() {
	nlohmann::json videoLoadConfig;
	std::stringstream storagePath;
	storagePath << m_VideoPath;
	std::ofstream fileOut(storagePath.str());

	videoLoadConfig["path"] = m_VideoPath;
	if (fileOut.is_open())
		fileOut << videoLoadConfig.dump();
	return 0;
}


std::string VideoRecording::getLoadedVideoName() {
	std::lock_guard<std::mutex> lock(mutex_);
	std::string filePath = m_VideoPath;
	while (filePath.find("/") != std::string::npos) {
		filePath.erase(0, filePath.find("/"));
	}

	Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Loaded video name: %s\n", filePath.c_str());
	return filePath;
}


int VideoRecording::deleteVideo(const std::string& filename) {
	Logger* logger = Logger::getLoggerInst();

	if (!std::filesystem::exists(VideoStoragePath)) {
		logger->log(Logger::LOG_LVL_ERROR, "Video storage path does not exist: %s\n", VideoStoragePath);
		return -1;
	}

	const std::string fullPath = (std::filesystem::path(VideoStoragePath) / filename).string();
	if (!std::filesystem::exists(fullPath)) {
		logger->log(Logger::LOG_LVL_ERROR, "Video file does not exist: %s\n", fullPath.c_str());
		return -1;
	}

	try {
		std::filesystem::remove(fullPath);
	} catch (const std::filesystem::filesystem_error& e) {
		logger->log(Logger::LOG_LVL_ERROR, "Failed to delete video file: %s\n", fullPath.c_str());
		return -1;
	}

	return 0;
}


int VideoRecording::loadFile(const std::string& filename) {
	Logger* logger = Logger::getLoggerInst();

	if (!std::filesystem::exists(VideoStoragePath)) {
		logger->log(Logger::LOG_LVL_ERROR, "Video storage path does not exist: %s\n", VideoStoragePath);
		return -1;
	}

	m_VideoPath = filename;

	// Accept names like "foo.MOV" from the client, but stored files are always ".rcv".
	std::filesystem::path outName(filename);
	const std::string ext = outName.extension().string();
	if (ext == ".MOV" || ext == ".mov") {
		outName.replace_extension(".rcv");
	} else if (ext.empty()) {
		outName.replace_extension(".rcv");
	}

	const std::string fullPath = (std::filesystem::path(VideoStoragePath) / outName).string();
	std::ifstream in(fullPath, std::ios::binary);
	if (!in.is_open()) {
		logger->log(Logger::LOG_LVL_ERROR, "Failed to open video file: %s\n", fullPath.c_str());
		return -1;
	}

	// File format:
	//   [u32 rows][u32 cols][u32 cvType][u32 payloadBytes][payload bytes] repeated.
	std::lock_guard<std::mutex> lock(mutex_);
	m_Video.clear();
	m_currentFrame = 0;

	constexpr uint32_t kMaxFrameBytes = 50u * 1024u * 1024u; // guard against corrupt headers
	constexpr uint32_t kMaxDim = 8192;                       // guard against corrupt headers
	while (true) {
		uint32_t rows = 0, cols = 0, type = 0, payload = 0;
		in.read(reinterpret_cast<char*>(&rows), sizeof(rows));
		if (in.eof()) {
			break;
		}
		in.read(reinterpret_cast<char*>(&cols), sizeof(cols));
		in.read(reinterpret_cast<char*>(&type), sizeof(type));
		in.read(reinterpret_cast<char*>(&payload), sizeof(payload));
		if (!in.good()) {
			logger->log(Logger::LOG_LVL_ERROR, "Corrupted video file (header read): %s\n", fullPath.c_str());
			return -1;
		}

		// Basic sanity checks to avoid cv::Mat throwing on invalid sizes/types.
		if (rows == 0 || cols == 0 || rows > kMaxDim || cols > kMaxDim || payload == 0 || payload > kMaxFrameBytes) {
			logger->log(Logger::LOG_LVL_ERROR, "Corrupted video file (invalid header) rows=%u cols=%u payload=%u: %s\n",
			            rows, cols, payload, fullPath.c_str());
			return -1;
		}

		const int iRows = static_cast<int>(rows);
		const int iCols = static_cast<int>(cols);
		const int iType = static_cast<int>(type);
		if (iRows <= 0 || iCols <= 0) {
			logger->log(Logger::LOG_LVL_ERROR, "Corrupted video file (invalid dims) rows=%u cols=%u: %s\n",
			            rows, cols, fullPath.c_str());
			return -1;
		}

		// Validate OpenCV type encoding before using it.
		const int depth = CV_MAT_DEPTH(iType);
		const int channels = CV_MAT_CN(iType);
		if (depth < 0 || depth > CV_16F || channels <= 0 || channels > CV_CN_MAX) {
			logger->log(Logger::LOG_LVL_ERROR, "Corrupted video file (invalid cv::Mat type=%u): %s\n",
			            type, fullPath.c_str());
			return -1;
		}

		// Best-effort sanity: payload should match the implied image size.
		const uint64_t expected = static_cast<uint64_t>(iRows) * static_cast<uint64_t>(iCols) * static_cast<uint64_t>(CV_ELEM_SIZE(iType));
		if (expected != payload) {
			logger->log(Logger::LOG_LVL_WARN, "Video frame payload mismatch (expected=%llu got=%u) in %s\n",
			            static_cast<unsigned long long>(expected), payload, fullPath.c_str());
		}

		std::vector<uint8_t> bytes(payload);
		in.read(reinterpret_cast<char*>(bytes.data()), payload);
		if (!in.good()) {
			logger->log(Logger::LOG_LVL_ERROR, "Corrupted video file (payload read): %s\n", fullPath.c_str());
			return -1;
		}

		try {
			cv::Mat frame(iRows, iCols, iType, bytes.data());
			m_Video.push_back(frame.clone());
		} catch (const cv::Exception& e) {
			logger->log(Logger::LOG_LVL_ERROR, "Failed to load frame from %s: %s\n", fullPath.c_str(), e.what());
			return -1;
		}
	}

	logger->log(Logger::LOG_LVL_INFO, "Loaded video file: %s (frames=%zu)\n", fullPath.c_str(), m_Video.size());
	return 0;
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

	// Stored format is always ".rcv", but accept a ".MOV" name from the client.
	std::filesystem::path outName(filename);
	const std::string ext = outName.extension().string();
	if (ext == ".MOV" || ext == ".mov") {
		outName.replace_extension(".rcv");
	} else if (ext.empty()) {
		outName.replace_extension(".rcv");
	} else if (ext != ".rcv") {
		Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Unsupported file format for saving: %s\n", filename.c_str());
		return -1;
	}

	const std::string fullPath = (std::filesystem::path(VideoStoragePath) / outName).string();
	std::ofstream out(fullPath, std::ios::binary | std::ios::trunc);
	if (!out.is_open()) {
		Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to open file for writing: %s\n", fullPath.c_str());
		return -1; // File open error
	}

	for (const auto& frame : m_Video) {
		if (frame.empty()) {
			continue;
		}
		const cv::Mat continuous = frame.isContinuous() ? frame : frame.clone();

		const uint32_t rows = static_cast<uint32_t>(continuous.rows);
		const uint32_t cols = static_cast<uint32_t>(continuous.cols);
		const uint32_t type = static_cast<uint32_t>(continuous.type());
		const uint64_t payload64 = static_cast<uint64_t>(continuous.total()) * continuous.elemSize();
		if (payload64 == 0 || payload64 > std::numeric_limits<uint32_t>::max()) {
			Logger::getLoggerInst()->log(Logger::LOG_LVL_WARN, "Skipping frame with invalid payload size (%llu bytes)\n",
			                            static_cast<unsigned long long>(payload64));
			continue;
		}
		const uint32_t payload = static_cast<uint32_t>(payload64);

		out.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
		out.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
		out.write(reinterpret_cast<const char*>(&type), sizeof(type));
		out.write(reinterpret_cast<const char*>(&payload), sizeof(payload));
		out.write(reinterpret_cast<const char*>(continuous.data), payload);
		if (!out.good()) {
			Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Error writing video recording: %s\n", fullPath.c_str());
			return -1;
		}
	}
	Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Video recording saved to file: %s\n", fullPath.c_str());
	return 0; // Success
}

}  // namespace Vision
