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
#include "app/video/VideoLaneNet.hpp"
#include "app/video/VideoStereo.hpp"

#include <opencv2/cudastereo.hpp>

namespace Modules {
class VisionControls : public Base, public Adapter::CameraAdapter {
public:
    /** @brief Construct the vision control module. */
    VisionControls(ModuleDefs::DeviceType moduleID, std::string name);

    /** @brief Destroy the vision control module. */
    ~VisionControls();

    /** @brief Initialize adapters, processing pipelines, and command bindings. */
    virtual int init(void) override;

    /** @brief Stop the module execution. */
    virtual int stop(void) override {
        return 0;
    }

    /** @brief Return this module as a camera adapter input. */
    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::CameraAdapter*>(this));
    }

    // virtual int moduleCommand_(char* pbuf, size_t len) override;

    /** @brief Handle mailbox command payloads dispatched to this module. */
    virtual int moduleCommand_(std::vector<char>& buffer) override;

    /** @brief Handle CLI commands sent to the vision module. */
    virtual int moduleCliCmd_(std::vector<std::string>& buffer) override;

    /** @brief Return runtime statistics for camera and calibration state. */
    virtual std::string readStats() override;

    /** @brief Start outbound video streaming. */
    void cmdStartStreamHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Stop outbound video streaming. */
    void cmdStopStreamHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Select active stream source and optional mode flags. */
    void cmdSelCameraStreamHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stream frame rate. */
    void cmdSetFpsHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stream JPEG quality. */
    void cmdSetQualityHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stereo minimum disparity. */
    void cmdSetMinDisparitiesHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stereo maximum disparity. */
    void cmdSetMaxDisparitiesHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stereo confidence threshold. */
    void cmdSetConfidenceThresholdHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stereo uniqueness ratio. */
    void cmdSetUniquenessRatioHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stereo P1 smoothness parameter. */
    void cmdSetP1Handler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set stereo P2 smoothness parameter. */
    void cmdSetP2Handler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set maximum depth range limit. */
    void cmdSetZMaxHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set minimum depth range limit. */
    void cmdSetZMinHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set depth agreement threshold. */
    void cmdSetDepthThresholdHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set minimum agreeing pixel count for depth filtering. */
    void cmdSetMinAgreeingPixelsHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Set color consistency threshold for depth filtering. */
    void cmdSetColorThresholdHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Read and return current camera/stereo parameters. */
    void cmdRdParamsHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Clear in-memory video recording state. */
    void cmdClrVideoRecHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Save recorded video to disk. */
    void cmdSaveVideoHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Enumerate stored video recordings. */
    void cmdLoadStoredVideosHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Load a selected video recording. */
    void cmdLoadSelectedVideoHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Delete a selected video recording. */
    void cmdDeleteVideoHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Enable or disable stereo calibration mode. */
    void cmdCalibrationSetStateHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Apply calibration configuration from payload JSON. */
    void cmdCalibrationWrtParamsHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Reset the calibration session. */
    void cmdCalibrationResetHandler(val_type_t val, const std::vector<char>& payload);
    /** @brief Persist calibration profile to disk. */
    void cmdCalibrationSaveHandler(val_type_t val, const std::vector<char>& payload);
    
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
        int confidenceType = 0;
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

    static constexpr int CAM_WIDTH  = 640;
    static constexpr int CAM_HEIGHT = 480;

    /** @brief Main worker thread procedure. */
    virtual void mainProc() override;

    /** @brief Periodic timer callback. */
    virtual void OnTimer(void) override;

    /** @brief Decode one JPEG frame entry into an OpenCV matrix. */
    void decodeJPEG(cv::Mat& frame, const Vision::VideoFrame& frameEntry);

    /** @brief Handle incoming Ethernet video/control payloads. */
    void onEthRecv(std::vector<char>& data);

    /** @brief Run stereo processing on a synchronized frame pair. */
    void processStereo(cv::Mat& disparityFrame, cv::Mat& pointCloudMat, std::pair<cv::Mat, cv::Mat>& stereoFramePair, cv::Matx44d& Q);

    /** @brief Run lane detection on the provided frame. */
    void processLaneDetection(cv::Mat& frame, cv::Mat& dst);

    /** @brief Save the current frame pair to disk when requested. */
    void storeFrame(const cv::Mat& frameL, const cv::Mat& frameR);

    /** @brief Save streaming and stereo settings to persistent storage. */
    static int saveStreamingProfile(CameraSettings& settings);

    /** @brief Load streaming and stereo settings from persistent storage. */
    static int loadStreamingProfile(CameraSettings& settings);

    /** @brief Clamp stereo settings to supported VPI/CUDA constraints. */
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

    cv::cuda::GpuMat m_dHoughLines;

    cv::dnn::Net m_Net;

    // Receive frame buffer
    cv::Mat m_ReceivedFrame;

    // Receive buffer
    std::vector<uint8_t> m_ReceivedFrameBuff;

    // Transmission port
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_TxAdapter{nullptr};

    // Training video input port
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_SimVideoAdapter{nullptr};

    // Video recorder
    Vision::VideoRecording m_VideoRecorder;

    std::unique_ptr<Vision::VideoStreamer> m_VideoStreamer{nullptr};

    std::unique_ptr<Vision::LaneNet> m_LaneNet{nullptr};

    std::unique_ptr<Vision::VideoStereo> m_VideoStereo{nullptr};

    // Stereo camera object
    std::unique_ptr<Devices::StereoCam> m_Cam{nullptr};
    
    // Video calibration
    Vision::VideoStereoCalib m_VideoCalib;

    // Incoming frame assembler
    Vision::VideoFrame m_StreamInFrame;

    // Flag marking whether we should save frames
    bool m_SaveFrames = false;

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
