#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <types.h>
#include <thread>
#include <optional>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "RcBase.hpp"
#include "Devices/network_interface/UdpServer.hpp"


namespace Modules {
class VisionControls : public Base, public Adapter::CameraAdapter {
public:
    VisionControls(int moduleID, std::string name);
    ~VisionControls() {}

    virtual int stop(void) override {
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::CameraAdapter*>(this));
    }

    virtual int moduleCommand(char* pbuf, size_t len) override {
        return 0;
    }

virtual int configurePipeline_(const std::string& host) override;
    
protected:
#pragma pack(push, 1)
    struct Metadata {
        uint32_t sequenceID;
        uint8_t  segmentID;
        uint8_t  numSegments;
        uint32_t totalLength;
        uint16_t length;
    };

    static constexpr long long MaxUDPLen      = 65507;
    static constexpr long long MaxPayloadSize = MaxUDPLen - sizeof(Metadata);

    struct FragmentPayload {
        struct Metadata metadata;
        uint8_t payload[MaxPayloadSize];
    };

    struct FramePackets {
        uint8_t numSegments = 0;
        uint64_t frameID = 0;   // ID tag
        std::unordered_map<int, std::vector<uint8_t>> segmentMap;
    };
#pragma pack(pop)
    enum {
        StreamInOff,
        StreamInOn,
    };

    typedef std::unordered_map<int, FramePackets> FrameEntry;

    // Parent main proc override
    virtual void mainProc() override;

    // Parent timer thread overridew
    virtual void OnTimer(void) override;

    // Frame transmission handler
    int transmitFrames(cv::Mat& frame);

    // Handles the decoding of JPEGs
    void decodeJPEG(cv::Mat& frame, FramePackets& frameEntry);

    // Receive frame handler
    void recvFrame(const uint8_t* pbuf, size_t length);

    struct StreamStatus {
        std::atomic<uint8_t> streamInStatus{StreamInOff};
        int streamInCounter = 0;  // Stream IN watchdog counter
    } m_StreamStats;

    // Opencv writer
    cv::VideoWriter m_Writer;

    // GStreamer pipeline
    std::string m_HostIP;

    // Pipeline mutex lock
    std::mutex m_HostIPMutex;

    // Video transmitter socket
    std::unique_ptr<Network::UdpServer> m_UdpSocket;

    //  Training video input socket (only over eth)
    std::unique_ptr<Network::UdpServer> m_UdpSimSocket;

    // Flag allowing the camera reader to feed the circular buffer
    std::atomic<bool> m_StreamerCanRun{false};

    // Frame received flag
    bool m_ReceivingFrame{false};

    // Receive frame buffer
    cv::Mat m_ReceivedFrame;

    // Receive buffer
    std::vector<uint8_t> m_ReceivedFrameBuff;

    // Received frame buffer
    Msg::CircularBuffer<FramePackets> m_FrameRecvBuff;

    // Received segment map
    FramePackets m_SegmentMap;

    // Frame ID
    uint32_t m_FrameID = 0;
};
}

#pragma endregion