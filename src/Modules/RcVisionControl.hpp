#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <types.h>
#include <thread>
#include <optional>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "RcBase.hpp"
#include "Devices/network_interface/UdpServer.hpp"
#include "Devices/GyroScope.hpp"

#include "app/VideoRecording.hpp"


namespace Modules {
class VisionControls : public Base, public Adapter::CameraAdapter {
public:
    VisionControls(int moduleID, std::string name);
    ~VisionControls() {}

    virtual int init(void) override;

    virtual int stop(void) override {
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::CameraAdapter*>(this));
    }

    // virtual int moduleCommand_(char* pbuf, size_t len) override;

    virtual int moduleCommand_(std::vector<char>& buffer) override;

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

#pragma pack(pop)
    enum {
        StreamCameraPairs,      // normal camera mode
        StreamSim,              // simulation camera mode
        StreamStereoCameraMono, // Compiled stereo image mode
        StreamModeMax           // max modes
    };

    // Camera module commands
    enum {
        CmdSetFrameRate,  // set camera frame rate
        CmdStartStream,   // start video stream
        CmdStopStream,    // stop video stream
        CmdStreamMode,    // start simulation video stream
        CmdSelCameraMode, // select camera mode
        CmdClrVideoRec,   // clear video recording buffer
        CmdSetVideoName,  // set video recording filename
        CmdSaveVideo      // save video recording to disks
    };

    // Camera process enums
    enum {
        CamModeNormal,
        CamModeDepth,
        CamModeDisparity,
        CamModeMax
    };

    struct CameraCommand {
        uint8_t command;
        val_type_t data;
        uint32_t payloadLen;
    } __attribute__((__packed__));

    struct CameraSettings {
        int frameRate = 30;
        int mode      = CamModeNormal;
        std::string videoName = "recording.MOV";
    } m_CamSettings;

    // Parent main proc override
    virtual void mainProc() override;

    // Parent timer thread overridew
    virtual void OnTimer(void) override;

    // Frame transmission handler
    void decodeJPEG(cv::Mat& frame, const Vision::VideoFrame& frameEntry);

    // Receive frame handler
    void recvFrame(std::vector<char>& data);

    // Frame processing handler
    void processFrame(cv::Mat& frame);

    // Stereo frame processing handler
    void processStereo(cv::Mat& stereoFrame, std::pair<cv::Mat, cv::Mat>& stereoFramePair);

    // Stereo frame processing handler
    void processFrame(std::pair<cv::Mat, cv::Mat>& stereoFrame);

    // Depth processing handler
    void processDepth(cv::Mat& frame);

    struct StreamStatus {
        std::atomic<uint8_t> streamInStatus{StreamCameraPairs};
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

    // Transmission port
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_TxAdapter{nullptr};

    // Reception port
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_RxAdapter{nullptr};

    // Video recorder
    Vision::VideoRecording m_VideoRecorder;

    // Incoming frame assembler
    Vision::VideoFrame m_StreamInFrame;

    // Frame ID
    uint32_t m_FrameID = 0;

    // DNN model
    cv::dnn::Net m_DnnNetDepth;
    cv::dnn::Net m_DnnNetLaneDetect;
};
}

#pragma endregion