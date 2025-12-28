#include "StereoCam.hpp"

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>  // close()

#include "utils/logger.hpp"

namespace Devices {

using namespace Argus;
using namespace EGLStream;

StereoCam::StereoCam(int deviceIDLeft, int deviceIDRight)
    : deviceIDLeft_(deviceIDLeft),
      deviceIDRight_(deviceIDRight) {}

StereoCam::~StereoCam() {}

int StereoCam::start(uint32_t w, uint32_t h, uint32_t fps) {
    w_   = static_cast<uint32_t>(w);
    h_   = static_cast<uint32_t>(h);
    fps_ = static_cast<uint32_t>(fps);

    provider_.reset(CameraProvider::create());
    iProvider_ = interface_cast<ICameraProvider>(provider_);
    if (!iProvider_) {
        return -1;
    }

    // Ensure transform session params are configured (you had this only in open()).
    if (!transformConfigured_) {
        NvBufSurfTransformConfigParams config = {};
        config.compute_mode = NvBufSurfTransformCompute_VIC;
        config.gpu_id = 0;
        config.cuda_stream = nullptr;

        if (NvBufSurfTransformSetSessionParams(&config) != NvBufSurfTransformError_Success) {
            return -1;
        }
        transformConfigured_ = true;
    }

    // Reset start barrier for a new run
    {
        std::lock_guard<std::mutex> lk(startMtx_);
        start_ = false;
    }

    // Prepare thread contexts
    threadCtx_[0].deviceIdx = deviceIDLeft_;
    threadCtx_[0].self = this;
    threadCtx_[0].camIndex = CamIdx::CamLeft;
    threadCtx_[0].iConsumer = iConsumers_[0];
    threadCtx_[0].frameBufferFd = &frameBufferFds_[0];
    threadCtx_[0].producerBuffer = &m_ProducerLeftBuffer;

    threadCtx_[1].deviceIdx = deviceIDRight_;
    threadCtx_[1].self = this;
    threadCtx_[1].camIndex = CamIdx::CamRight;
    threadCtx_[1].iConsumer = iConsumers_[1];
    threadCtx_[1].frameBufferFd = &frameBufferFds_[1];
    threadCtx_[1].producerBuffer = &m_ProducerRightBuffer;

    if (m_Cam0CaptureThread_.start(&StereoCam::captureThreadEntry, &threadCtx_[0], 1, true) != 0) {
        throw std::runtime_error("Failed to start camera 0 capture thread");
    }

    if (m_Cam1CaptureThread_.start(&StereoCam::captureThreadEntry, &threadCtx_[1], 2, true) != 0) {
        m_Cam0CaptureThread_.join();
        throw std::runtime_error("Failed to start camera 1 capture thread");
    }

    if (m_CamSynchronizer.start(&StereoCam::synchThreadEntry, this, 3, true) != 0) {
        m_Cam0CaptureThread_.join();
        m_Cam1CaptureThread_.join();
        throw std::runtime_error("Failed to start camera synchronizer thread");
    }

    return 0;
}


int StereoCam::openCamera_(size_t index, uint32_t sensorId) {
    std::vector<CameraDevice*> devices;
    iProvider_->getCameraDevices(&devices);
    if (sensorId >= devices.size()) return -1;

    // Choose an explicit sensor mode that can actually sustain the requested FPS.
    // If Argus picks a mode that allows long exposures (low-light), you can end up with ~20 FPS.
    CameraDevice* device = devices[sensorId];
    SensorMode* selectedMode = nullptr;
    Range<uint64_t> selectedFdRange(0, 0);
    Range<uint64_t> selectedExpRange(0, 0);
    {
        ICameraProperties* iProps = interface_cast<ICameraProperties>(device);
        if (iProps) {
            std::vector<SensorMode*> modes;
            iProps->getAllSensorModes(&modes);

            const uint64_t targetFrameDuration = 1'000'000'000ULL / std::max<uint32_t>(1, fps_);

            auto modeSupports = [&](SensorMode* mode) -> bool {
                ISensorMode* iMode = interface_cast<ISensorMode>(mode);
                if (!iMode) return false;
                const Size2D<uint32_t> res = iMode->getResolution();
                if (res.width() != w_ || res.height() != h_) return false;
                const Range<uint64_t> fd = iMode->getFrameDurationRange();
                return (fd.min() <= targetFrameDuration) && (targetFrameDuration <= fd.max());
            };

            // Prefer an exact resolution match that supports the requested frame duration.
            for (SensorMode* m : modes) {
                if (modeSupports(m)) {
                    selectedMode = m;
                    ISensorMode* iMode = interface_cast<ISensorMode>(m);
                    selectedFdRange = iMode->getFrameDurationRange();
                    selectedExpRange = iMode->getExposureTimeRange();
                    break;
                }
            }

            // If no exact match, fall back to the mode with the smallest min frame duration (highest FPS).
            if (!selectedMode && !modes.empty()) {
                selectedMode = *std::min_element(
                    modes.begin(), modes.end(),
                    [](SensorMode* a, SensorMode* b) {
                        ISensorMode* ia = interface_cast<ISensorMode>(a);
                        ISensorMode* ib = interface_cast<ISensorMode>(b);
                        if (!ia || !ib) return false;
                        return ia->getFrameDurationRange().min() < ib->getFrameDurationRange().min();
                    });
                if (ISensorMode* iMode = interface_cast<ISensorMode>(selectedMode)) {
                    selectedFdRange = iMode->getFrameDurationRange();
                    selectedExpRange = iMode->getExposureTimeRange();
                }
            }
        }
    }

    sessions_[index].reset(iProvider_->createCaptureSession(device));
    iSessions_[index] = interface_cast<ICaptureSession>(sessions_[index]);
    if (!iSessions_[index]) return -1;

    UniqueObj<OutputStreamSettings> settings(iSessions_[index]->createOutputStreamSettings(STREAM_TYPE_EGL));
    auto* iEglSettings = interface_cast<IEGLOutputStreamSettings>(settings);
    if (!iEglSettings) return -1;

    IEGLOutputStreamSettings *iStreamSettings =
        interface_cast<IEGLOutputStreamSettings>(settings);
    if (iStreamSettings) {
        iStreamSettings->setMetadataEnable(true);
    }

    iEglSettings->setResolution(Size2D<uint32_t>(w_, h_));
    iEglSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEglSettings->setEGLDisplay(EGL_NO_DISPLAY);
    iEglSettings->setMode(EGL_STREAM_MODE_MAILBOX);
    iEglSettings->setMetadataEnable(true);
    // iEglSettings->setMetadataEnabled(true);

    streams_[index].reset(iSessions_[index]->createOutputStream(settings.get()));

    consumers_[index].reset(EGLStream::FrameConsumer::create(streams_[index].get()));
    iConsumers_[index] = interface_cast<EGLStream::IFrameConsumer>(consumers_[index]);
    if (!iConsumers_[index]) return -1;

    requests_[index].reset(iSessions_[index]->createRequest(CAPTURE_INTENT_VIDEO_RECORD));
    auto* iRequest = interface_cast<IRequest>(requests_[index]);
    if (!iRequest) return -1;

    iRequest->enableOutputStream(streams_[index].get());

    auto* iSource = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    if (!iSource) return -1;

    const uint64_t frameDuration = 1'000'000'000ULL / fps_;

    if (selectedMode) {
        iSource->setSensorMode(selectedMode);
    }
    iSource->setFrameDurationRange(Range<uint64_t>(frameDuration, frameDuration));

    // Cap exposure time to the frame period to prevent long exposures from lowering FPS.
    // (This API is exposed on ISourceSettings in this Argus SDK.)
    {
        const uint64_t expMin = (selectedExpRange.min() > 0) ? selectedExpRange.min() : 1ULL;
        const uint64_t expMaxMode = (selectedExpRange.max() > 0) ? selectedExpRange.max() : frameDuration;
        const uint64_t expMax = std::min<uint64_t>(expMaxMode, frameDuration);
        if (expMin <= expMax) {
            iSource->setExposureTimeRange(Range<uint64_t>(expMin, expMax));
        }
    }

    if (iSessions_[index]->repeat(requests_[index].get()) != STATUS_OK) return -1;

    started_ = true;
    return 0;
}

int StereoCam::close() {
    for (size_t i = 0; i < kNumCameras; ++i) {
        if (bgrSurfaces_[i] && bgrMapped_[i]) {
            NvBufSurfaceUnMap(bgrSurfaces_[i], 0, 0);
            bgrMapped_[i] = false;
            bgrMappedPtr_[i] = nullptr;
            bgrPitch_[i] = 0;
        }
        if (bgrSurfaces_[i]) {
            NvBufSurfaceDestroy(bgrSurfaces_[i]);
            bgrSurfaces_[i] = nullptr;
        }
        if (frameSurfaces_[i]) {
            NvBufSurfaceDestroy(frameSurfaces_[i]);
            frameSurfaces_[i] = nullptr;
        }
        if (frameBufferFds_[i] >= 0) {
            ::close(frameBufferFds_[i]);     // IMPORTANT: close DMABUF fd
            frameBufferFds_[i] = -1;
        }

        iConsumers_[i] = nullptr;
        iSessions_[i] = nullptr;
        consumers_[i].reset();
        requests_[i].reset();
        streams_[i].reset();
        sessions_[i].reset();
    }

    iProvider_ = nullptr;
    provider_.reset();
    started_ = false;
    return 0;
}

int StereoCam::read(cv::Mat& leftBgr, cv::Mat& rightBgr) {
    if (!started_) return -1;

    // If you can, replace this with a blocking condition variable from your CircularBuffer.
    while (m_StereoBuffer.isEmpty()) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        if (!m_ThreadCanRun.load()) return -1;
    }

    auto& frames = m_StereoBuffer.getHead();

    // No deep clone here. You already clone in readCamera_ after mapping.
    leftBgr  = frames.first;
    rightBgr = frames.second;

    m_StereoBuffer.pop();
    return 0;
}


int StereoCam::readCamera_(size_t index, cv::Mat& outBgr, uint64_t& outTimestampNs) {
    if (index >= kNumCameras || !iConsumers_[index]) return -1;

    Status status = STATUS_OK;
    UniqueObj<EGLStream::Frame> frame(iConsumers_[index]->acquireFrame(TIMEOUT_INFINITE, &status));
    if (!frame || status != STATUS_OK) return -1;

    IArgusCaptureMetadata *iArgusCaptureMetadata = interface_cast<IArgusCaptureMetadata>(frame);
    if (iArgusCaptureMetadata) {
        CaptureMetadata *metadata = iArgusCaptureMetadata->getMetadata();
        ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(metadata);
        outTimestampNs = iMetadata->getSensorTimestamp();  // Gather sensor timestamp
        // std::cout << "Camera " << index << " timestamp (ns): " << outTimestampNs << std::endl;
    } else {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
        "Failed to get IArgusCaptureMetadata interface for camera %zu\n",
        index);
    }

    auto* iFrame = interface_cast<EGLStream::IFrame>(frame);
    if (!iFrame) return -1;

    EGLStream::Image* image = iFrame->getImage();
    if (!image) return -1;

    auto* iNative = interface_cast<EGLStream::NV::IImageNativeBuffer>(image);
    if (!iNative) return -1;

    // Create one persistent NvBuffer per camera, then reuse it each frame
    if (frameBufferFds_[index] < 0) {
        int fd = iNative->createNvBuffer(
            Size2D<uint32_t>(w_, h_),
            NVBUF_COLOR_FORMAT_NV12,
            NVBUF_LAYOUT_PITCH,
            EGLStream::NV::ROTATION_0,
            &status);

        if (fd < 0 || status != STATUS_OK) return -1;

        frameBufferFds_[index] = fd;

        if (NvBufSurfaceFromFd(frameBufferFds_[index],
                               reinterpret_cast<void**>(&frameSurfaces_[index])) != 0) {
            ::close(frameBufferFds_[index]);
            frameBufferFds_[index] = -1;
            return -1;
        }
    }

    if (iNative->copyToNvBuffer(frameBufferFds_[index]) != STATUS_OK) return -1;

    NvBufSurface* srcSurface = frameSurfaces_[index];
    if (!srcSurface) return -1;

    if (!bgrSurfaces_[index]) {
        NvBufSurfaceCreateParams params = {};
        params.gpuId = 0;
        params.width = w_;
        params.height = h_;
        params.colorFormat = NVBUF_COLOR_FORMAT_BGRx;
        params.layout = NVBUF_LAYOUT_PITCH;
        params.memType = NVBUF_MEM_DEFAULT;

        if (NvBufSurfaceCreate(&bgrSurfaces_[index], 1, &params) != 0) return -1;
    }

    NvBufSurface* dstSurface = bgrSurfaces_[index];

    NvBufSurfTransformParams transformParams = {};
    transformParams.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
    // Fastest is usually nearest; we're not scaling here.
    transformParams.transform_filter = NvBufSurfTransformInter_Nearest;

    if (NvBufSurfTransform(srcSurface, dstSurface, &transformParams) != NvBufSurfTransformError_Success) return -1;

    // Map once per camera and keep it mapped; just sync per-frame.
    if (!bgrMapped_[index]) {
        if (NvBufSurfaceMap(dstSurface, 0, 0, NVBUF_MAP_READ) != 0) return -1;
        bgrMappedPtr_[index] = dstSurface->surfaceList[0].mappedAddr.addr[0];
        bgrPitch_[index] = dstSurface->surfaceList[0].pitch;
        if (!bgrMappedPtr_[index] || bgrPitch_[index] == 0) {
            NvBufSurfaceUnMap(dstSurface, 0, 0);
            bgrMappedPtr_[index] = nullptr;
            bgrPitch_[index] = 0;
            return -1;
        }
        bgrMapped_[index] = true;
    }

    if (NvBufSurfaceSyncForCpu(dstSurface, 0, 0) != 0) {
        return -1;
    }

    // Copy into a preallocated cv::Mat (avoid per-frame allocations).
    outBgr.create(static_cast<int>(h_), static_cast<int>(w_), CV_8UC4);
    const auto* src = static_cast<const uint8_t*>(bgrMappedPtr_[index]);
    const uint32_t pitch = bgrPitch_[index];
    const uint32_t rowBytes = w_ * 4;
    for (uint32_t y = 0; y < h_; ++y) {
        std::memcpy(outBgr.ptr(static_cast<int>(y)), src + static_cast<std::size_t>(y) * pitch, rowBytes);
    }
    return 0;
}


void StereoCam::streamProducer(CamIdx idx, Msg::CircularBuffer<FrameObject>& producerBuffer) {
    // Small pool so we can reuse allocations without overwriting frames still queued.
    constexpr std::size_t kPoolSize = 6;
    std::array<cv::Mat, kPoolSize> framePool;
    std::size_t poolIdx = 0;

    uint64_t timestampNs = 0;

    // Start barrier: wait until consumer signals start_
    {
        std::unique_lock<std::mutex> lk(startMtx_);
        startCv_.wait(lk, [this] { return start_ || !m_ThreadCanRun.load(); });
    }

        if (openCamera_(static_cast<size_t>(idx), threadCtx_[static_cast<size_t>(idx)].deviceIdx) != 0) {
            throw std::runtime_error("Failed to open camera");
        }

        while (m_ThreadCanRun.load()) {
            cv::Mat& frameSlot = framePool[poolIdx];
            poolIdx = (poolIdx + 1) % kPoolSize;

            if (readCamera_(static_cast<size_t>(idx), frameSlot, timestampNs) != 0) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
                                         "Failed to read frame from camera %d\n",
                                         static_cast<int>(idx));
            continue;
        }

        FrameObject frameObj;
            frameObj.frame = frameSlot;  // shallow copy; backing store is the pool slot
        frameObj.timestampNs = timestampNs;

        producerBuffer.push(frameObj);
    }
}


void StereoCam::streamConsumer() {
    const uint64_t THRESH_NS = 5ULL * 1000 * 1000; // 5 ms

    // Release producers at (nearly) the same time
    {
        std::lock_guard<std::mutex> lk(startMtx_);
        start_ = true;
    }
    startCv_.notify_all();

    while (m_ThreadCanRun.load()) {
        if (m_ProducerLeftBuffer.isEmpty() || m_ProducerRightBuffer.isEmpty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        const FrameObject& left  = m_ProducerLeftBuffer.getHead();
        const FrameObject& right = m_ProducerRightBuffer.getHead();

        // Check timestamp alignment
        int64_t timeDiff = static_cast<int64_t>(left.timestampNs) - static_cast<int64_t>(right.timestampNs);

        const uint64_t absDiff = static_cast<uint64_t>(timeDiff < 0 ? -timeDiff : timeDiff);
        if (absDiff <= THRESH_NS) {
            // Aligned: pop both and publish.
            FrameObject leftOut = left;
            FrameObject rightOut = right;
            m_ProducerLeftBuffer.pop();
            m_ProducerRightBuffer.pop();
            m_StereoBuffer.push({leftOut.frame, rightOut.frame});
        } else {
            // Not aligned: drop the older frame (smaller timestamp) to catch up.
            if (timeDiff < 0) {
                m_ProducerLeftBuffer.pop();
            } else {
                m_ProducerRightBuffer.pop();
            }
        }
    }
}


void* StereoCam::captureThreadEntry(void* userData) {
    auto* ctx = static_cast<StereoCam::ThreadContext*>(userData);
    if (!ctx || !ctx->self) return nullptr;

    ctx->self->streamProducer(ctx->camIndex, *ctx->producerBuffer);
    return nullptr;
}


void* StereoCam::synchThreadEntry(void* userData) {
    auto* self = static_cast<StereoCam*>(userData);
    if (!self) return nullptr;

    self->streamConsumer();
    return nullptr;
}

} // namespace Devices
