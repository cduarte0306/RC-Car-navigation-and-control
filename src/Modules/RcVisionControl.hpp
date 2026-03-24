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
#include "Devices/StereoCam.hpp"
#include "Devices/GyroScope.hpp"

#include "app/video/VideoRecording.hpp"
#include "app/video/VideoStereoCalibration.hpp"
#include "app/video/VideoStreamer.hpp"

#include <opencv2/cudastereo.hpp>
#include <vpi/OpenCVInterop.hpp>
  
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>

namespace Modules {
class VisionControls : public Base, public Adapter::CameraAdapter {
public:
    VisionControls(int moduleID, std::string name);
    ~VisionControls();

    virtual int init(void) override;

    virtual int stop(void) override {
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::CameraAdapter*>(this));
    }

    // virtual int moduleCommand_(char* pbuf, size_t len) override;

    virtual int moduleCommand_(std::vector<char>& buffer) override;

    // Command handlers
    virtual std::string readStats() override;
    
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
        StreamCameraSource,  // normal camera mode
        StreamSimSource,     // simulation camera mode
        StreamMaxSources
    };

    // Camera module commands
    enum {
        CmdStartStream,           // start video stream
        CmdStopStream,            // stop video stream
        CmdSelCameraStream,       // select camera stream (Normal or training)
        CmdSetFps,                // set streaming parameters (host IP, port, etc)
        CmdSetQuality,            // Set the stream compresison quality
        
        // Stereo parameters
        CmdSetMinDisparities,      // Set maximum disparities
        CmdSetMaxDisparities,
        CmdSetConfidenceThreshold,
        CmdSetUniquenessRatio,    // Sets the uniqueness ratio
        CmdSetP1,                 // Sets StereoSGM P1
        CmdSetP2,                 // Sets StereoSGM P2
        CmdSetZMax,
        CmdSetZMin,
        CmdSetDepthThreshold,
        CmdSetMinAgreeingPixels,
        CmdSetColorThreshold,

        CmdRdParams,              // Command to read all configured parameteres
        CmdClrVideoRec,           // clear video recording buffer
        CmdSaveVideo,             // save video recording to disks
        CmdLoadStoredVideos,      // Read stored videos from disk
        CmdLoadSelectedVideo,     // load selected video from disk
        CmdDeleteVideo,           // Delete video
        CmdCalibrationSetState,   // Start video calibration
        CmdCalibrationWrtParams,  // Sets calibration paramters
        CmdCalibrationReset,      // Reset calibration
        CmdCalibrationSave,       // Save calibration to disk
    };

    // Camera process enums
    enum {
        CamModeNormal,
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
        int quality = 100;
        int numDisparities = 128;
        int numBlocks = 15;

        int preFilterType;
        int preFilterSize;
        int preFilterCap;
        int textureThreshold;
        int uniquenessRatio;
        int speckleWindowSize;
        int speckleRange;
        int disp12MaxDiff;
        int p1 = 0;
        int p2 = 0;
        int sgmMode = cv::cuda::StereoSGM::MODE_HH4;

        // VPI stereo runtime parameters (windowSize intentionally not user-configurable)
        int maxDisparity = 0;
        int minDisparity = 0;
        int confidenceThreshold = 32767;
        int confidenceType = VPI_STEREO_CONFIDENCE_ABSOLUTE;
        int vpiQuality = 1;
        int p2Alpha = 0;
        float uniqueness = -1.0f;
        int numPasses = 3;
        float depthThreshold = 0.01f;
        int   minAgreeingPixels = 20;
        float colorThreshold = 30.0f;

        float zMax = 10;
        float zMin = 0;

        int mode      = CamModeNormal;
        std::atomic<uint8_t> streamSelection{StreamCameraSource};
        std::atomic<bool> calibrationMode{false};
        std::string videoName = "recording.MOV";
        cv::Size chessboardSize{9, 6}; // inner corners (cols, rows)
    } m_CamSettings;

    static constexpr int CAM_WIDTH  = 1280;
    static constexpr int CAM_HEIGHT = 720;

    // Parent main proc override
    virtual void mainProc() override;

    // Parent timer thread overridew
    virtual void OnTimer(void) override;

    // Frame transmission handler
    void decodeJPEG(cv::Mat& frame, const Vision::VideoFrame& frameEntry);

    // Receive frame handler
    void onEthRecv(std::vector<char>& data);

    // Stereo frame processing handler
    void processStereo(cv::Mat& disparityFrame, cv::Mat& pointCloudMat, std::pair<cv::Mat, cv::Mat>& stereoFramePair, cv::Matx44d& Q);

    // Lane detection
    void processLaneDetection(cv::Mat& frame, cv::Mat& dst);

    // Save the streaming profile parameters
    static int saveStreamingProfile(CameraSettings& settings);

    // Load all settings
    static int loadStreamingProfile(CameraSettings& settings);

    // Clamp settings to VPI CUDA stereo constraints.
    static void sanitizeVpiStereoSettings(CameraSettings& settings);

    // Camera source stream handler
    // void processCameraSource(Devices::StereoCam& cam);

    struct StreamStatus {
        std::atomic<uint8_t> streamInStatus{StreamCameraSource};
        int streamInCounter = 0;  // Stream IN watchdog counter
    } m_StreamStats;

    // Opencv writer
    cv::VideoWriter m_Writer;

    // GStreamer pipeline
    std::string m_HostIP;

    // Pipeline mutex lock
    std::mutex m_HostIPMutex;

    // Stereo object mutex
    std::mutex m_StereoMutex;

    // Video transmitter socket
    std::unique_ptr<Network::UdpServer> m_UdpSocket;

    // Flag allowing the camera reader to feed the circular buffer
    std::atomic<bool> m_StreamerCanRun{false};

    // Frame received flag
    bool m_ReceivingFrame{false};

    VPIStream  m_VpiStream = nullptr;
    VPIPayload m_Stereo    = nullptr;

    VPIImage m_ImgL         = nullptr;
    VPIImage m_ImgR         = nullptr;
    VPIImage m_ImgL_8u      = nullptr;
    VPIImage m_ImgR_8u      = nullptr;
    VPIImage m_ImgL_270p    = nullptr;
    VPIImage m_ImgR_270p    = nullptr;
    VPIImage m_Disparity    = nullptr;
    VPIImage m_ConfidenceMap = nullptr;

    const int w_ = 640;
    const int h_ = 480;

    uint64_t backend_ = 0;

    VPIStereoDisparityEstimatorCreationParams m_CreateParams;

    VPIStereoDisparityEstimatorParams m_StereoParams;

    VPIConvertImageFormatParams m_ConvParams;

    cv::cuda::GpuMat m_dHoughLines;

    // Receive frame buffer
    cv::Mat m_ReceivedFrame;

    // Receive buffer
    std::vector<uint8_t> m_ReceivedFrameBuff;

    // Transmission port
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_TxAdapter{nullptr};

    // Reception port
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_EthAdapter{nullptr};

    // Training video input port
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_SimVideoAdapter{nullptr};

    // Video recorder
    Vision::VideoRecording m_VideoRecorder;

    std::unique_ptr<Vision::VideoStreamer> m_VideoStreamer{nullptr};

    // Stereo camera object
    std::unique_ptr<Devices::StereoCam> m_Cam{nullptr};
    
    // Video calibration
    Vision::VideoStereoCalib m_VideoCalib;

    // Incoming frame assembler
    Vision::VideoFrame m_StreamInFrame;

    // Frame ID
    uint32_t m_FrameID = 0;

    // Name of the currently-downloaded (sim) video, derived from incoming packet metadata.
    std::string m_LastIncomingVideoName;

    // DNN model
    cv::dnn::Net m_DnnNetDepth;
    cv::dnn::Net m_DnnNetLaneDetect;
};
}

#pragma endregion
