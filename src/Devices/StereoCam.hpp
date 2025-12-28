#pragma once

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <opencv2/opencv.hpp>
#include <array>
#include <mutex>
#include <condition_variable>
#include "lib/Thread.hpp"
#include "Modules/RcMessageLib.hpp"

struct NvBufSurface;



namespace Devices {
class StereoCam {
public:
    StereoCam(int deviceIDLeft, int deviceIDRight);
    ~StereoCam();

    /**
     * @brief Starts the stereo camera devices
     * 
     * @param w 
     * @param h 
     * @param fps
     * @return int 
     */ 
    int start(uint32_t w, uint32_t h, uint32_t fps);

    /**
     * @brief 
     * 
     * @return int 
     */
    int close();

    /**
     * @brief Reads synchronized frames from both cameras
     * 
     * @param leftBgr Left camera BGR frame output
     * @param rightBgr Right camera BGR frame output
     * @return int 
     */
    int read(cv::Mat& leftBgr, cv::Mat& rightBgr);

private:
    enum class CamIdx : uint8_t {
      CamLeft,
      CamRight  
    };

    struct FrameObject {
      cv::Mat frame;
      uint64_t timestampNs;
    };

    struct ThreadContext {
      StereoCam* self = nullptr;
      int deviceIdx = -1;
      CamIdx camIndex = CamIdx::CamLeft;
      EGLStream::IFrameConsumer* iConsumer = nullptr;
      int* frameBufferFd;
      Msg::CircularBuffer<FrameObject>* producerBuffer = nullptr;
    };

    /**
     * @brief Open a camera device 
     * 
     * @param index 
     * @param sensorId 
     * @return int 
     */
    int open_(int index, int sensorId);
    int openCamera_(size_t index, uint32_t sensorId);
    int readCamera_(size_t index, cv::Mat& outBgr, uint64_t& outTimestampNs);

    /**
     * @brief Convert NvBufSurface to cv::Mat in BGR format
     * 
     * @param dstSurface 
     * @param outBgr 
     * @return int 
     */
    void cam0CaptureProducer();

    /**
     * @brief Convert NvBufSurface to cv::Mat in BGR format
     * 
     * @param dstSurface 
     * @param outBgr 
     * @return int 
     */
    void cam1CaptureProducer();

    /**
     * @brief Camera stream producer thread
     * 
     * @param idx 
     * @param producerBuffer 
     */
    void streamProducer(CamIdx idx, Msg::CircularBuffer<FrameObject>& producerBuffer);
    void streamConsumer(void);
    static void* captureThreadEntry(void* userData);
    static void* synchThreadEntry(void* userData);
    
    int deviceIDLeft_;
    int deviceIDRight_;

    uint32_t w_;
    uint32_t h_;
    uint32_t fps_;
    bool started_ = false;

    static constexpr size_t kNumCameras = 2;
    Argus::UniqueObj<Argus::CameraProvider> provider_;
    std::array<Argus::UniqueObj<Argus::CaptureSession>, kNumCameras> sessions_;
    std::array<Argus::UniqueObj<Argus::OutputStream>, kNumCameras> streams_;
    std::array<Argus::UniqueObj<Argus::Request>, kNumCameras> requests_;
    std::array<Argus::UniqueObj<EGLStream::FrameConsumer>, kNumCameras> consumers_;

    Argus::ICameraProvider* iProvider_ = nullptr;
    std::array<Argus::ICaptureSession*, kNumCameras> iSessions_ = {nullptr, nullptr};
    std::array<EGLStream::IFrameConsumer*, kNumCameras> iConsumers_ = {nullptr, nullptr};
    std::array<int, kNumCameras> frameBufferFds_ = {-1, -1};
    std::array<NvBufSurface*, kNumCameras> frameSurfaces_ = {nullptr, nullptr};
    std::array<NvBufSurface*, kNumCameras> bgrSurfaces_ = {nullptr, nullptr};
    bool transformConfigured_ = false;

    std::array<ThreadContext, kNumCameras> threadCtx_;

    // Cam frame producer threads
    Lib::Thread m_Cam0CaptureThread_;
    Lib::Thread m_Cam1CaptureThread_;
    Lib::Thread m_CamSynchronizer;

    // Producer buffers
    Msg::CircularBuffer<FrameObject> m_ProducerLeftBuffer{100};
    Msg::CircularBuffer<FrameObject> m_ProducerRightBuffer{100};
    Msg::CircularBuffer<std::pair<cv::Mat, cv::Mat>> m_StereoBuffer{100};

    std::mutex startMtx_;
    std::condition_variable startCv_;
    bool start_ = false;
    bool m_ThreadCanRun{true};
};
};

#pragma endregion
