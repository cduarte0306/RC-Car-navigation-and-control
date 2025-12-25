#include "app/VideoFrame.hpp"

VideoFrame::VideoFrame(std::size_t id) : frameID_(id) {}


void VideoFrame::setExpected(std::size_t expectedSegments, std::size_t expectedTotalLength) noexcept {
    expectedSegments_ = expectedSegments;
    expectedTotalLength_ = expectedTotalLength;
}


void VideoFrame::append(int id, const uint8_t* src, std::size_t length) {
    if (!src || length == 0) return;
    m_FrameSegMap[id].insert(m_FrameSegMap[id].end(), src, src + length);
}


void VideoFrame::append(const uint8_t* src, std::size_t length) {
    append(0, src, length);
}


void VideoFrame::append(int id, const std::vector<uint8_t>& buffer) {
    if (buffer.empty()) return;
    m_FrameSegMap[id].insert(m_FrameSegMap[id].end(), buffer.begin(), buffer.end());
}


void VideoFrame::append(const std::vector<uint8_t>& buffer) {
    append(0, buffer);
}


std::size_t VideoFrame::id() const noexcept {
    return frameID_;
}


const std::vector<uint8_t> VideoFrame::bytes() const noexcept {
    std::vector<uint8_t> data;
    if (expectedTotalLength_ > 0) {
        data.reserve(expectedTotalLength_);
    }

    // Enforce ordered assembly so the encoded JPEG is correct
    if (expectedSegments_ > 0 && m_FrameSegMap.size() != expectedSegments_) {
        return data;  // incomplete
    }

    for (const auto& [segID, segData] : m_FrameSegMap) {
        (void)segID;
        data.insert(data.end(), segData.begin(), segData.end());
    }
    return data;
}


std::size_t VideoFrame::size() const noexcept {
    std::size_t total = 0;
    for (const auto& seg : m_FrameSegMap) {
        total += seg.second.size();
    }
    return total;
}


void VideoFrame::reset(std::size_t newId) {
    frameID_ = newId;
    expectedSegments_ = 0;
    expectedTotalLength_ = 0;
    m_FrameSegMap.clear();
}
