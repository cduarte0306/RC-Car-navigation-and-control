#include "VideoStereo.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudastereo.hpp>

#include <vpi/OpenCVInterop.hpp>

#include "utils/logger.hpp"
#include "cudaVision.hpp"


#define CHECK_STATUS(STMT)                                                                  \
    do                                                                                      \
    {                                                                                       \
        VPIStatus status = (STMT);                                                          \
        if (status != VPI_SUCCESS)                                                          \
        {                                                                                   \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];                                     \
            vpiGetLastStatusMessage(buffer, sizeof(buffer));                                \
            std::ostringstream ss;                                                          \
            ss << "line " << __LINE__ << " " << vpiStatusGetName(status) << ": " << buffer; \
            throw std::runtime_error(ss.str());                                             \
        }                                                                                   \
    } while (0)


namespace Vision {
namespace {
static int clampInt(int value, int lo, int hi) {
    return std::max(lo, std::min(value, hi));
}

static constexpr int kHardMaxDisparity   = 64;
static constexpr int kPayloadMaxDisparity = 255;
}


void VideoStereo::sanitizeSettings(Settings& s) {
    s.numDisparities = kHardMaxDisparity;
    s.minDisparity   = clampInt(s.minDisparity, 0, s.numDisparities);

    if (s.p1 <= 0 || s.p1 > 255) s.p1 = 3;
    if (s.p2 <= 0 || s.p2 > 255) s.p2 = 48;
    s.p1 = clampInt(s.p1, 1, 255);
    s.p2 = clampInt(s.p2, s.p1, 255);

    s.uniquenessRatio = clampInt(s.uniquenessRatio, 0, 30);

    if (s.maxDisparity != 0 && s.maxDisparity != kPayloadMaxDisparity) {
        s.maxDisparity = 0;
    }
    s.confidenceThreshold = clampInt(s.confidenceThreshold, 0, 65535);
    s.vpiQuality = clampInt(s.vpiQuality, 1, 8);
    s.confidenceType = clampInt(s.confidenceType,
                                static_cast<int>(VPI_STEREO_CONFIDENCE_ABSOLUTE),
                                static_cast<int>(VPI_STEREO_CONFIDENCE_INFERENCE));
    if (!(s.p2Alpha == 0 || s.p2Alpha == 1 || s.p2Alpha == 2 || s.p2Alpha == 4 || s.p2Alpha == 8)) {
        s.p2Alpha = 0;
    }
    if (s.uniqueness < 0.0f) {
        s.uniqueness = -1.0f;
    } else if (s.uniqueness > 1.0f) {
        s.uniqueness = 1.0f;
    }
    s.numPasses = clampInt(s.numPasses, 1, 3);
}


VideoStereo::VideoStereo() {
    vpiInitConvertImageFormatParams(&m_ConvParams);
    vpiInitStereoDisparityEstimatorParams(&m_StereoParams);
    vpiInitStereoDisparityEstimatorCreationParams(&m_CreateParams);
}


VideoStereo::~VideoStereo() {
    if (m_ImgL)          { vpiImageDestroy(m_ImgL);          m_ImgL          = nullptr; }
    if (m_ImgR)          { vpiImageDestroy(m_ImgR);          m_ImgR          = nullptr; }
    if (m_ImgL_8u)       { vpiImageDestroy(m_ImgL_8u);       m_ImgL_8u       = nullptr; }
    if (m_ImgR_8u)       { vpiImageDestroy(m_ImgR_8u);       m_ImgR_8u       = nullptr; }
    if (m_ImgL_Scaled)   { vpiImageDestroy(m_ImgL_Scaled);   m_ImgL_Scaled   = nullptr; }
    if (m_ImgR_Scaled)   { vpiImageDestroy(m_ImgR_Scaled);   m_ImgR_Scaled   = nullptr; }
    if (m_Disparity)     { vpiImageDestroy(m_Disparity);     m_Disparity     = nullptr; }
    if (m_ConfidenceMap) { vpiImageDestroy(m_ConfidenceMap); m_ConfidenceMap = nullptr; }
    if (m_StereoPayload) { vpiPayloadDestroy(m_StereoPayload); m_StereoPayload = nullptr; }
    if (m_VpiStream)     { vpiStreamDestroy(m_VpiStream);    m_VpiStream     = nullptr; }
}


int VideoStereo::init(int camWidth, int camHeight, int processWidth, int processHeight) {
    Logger* logger = Logger::getLoggerInst();
    m_CamWidth  = camWidth;
    m_CamHeight = camHeight;
    m_Width  = processWidth;
    m_Height = processHeight;

    CHECK_STATUS(vpiStreamCreate(0, &m_VpiStream));
    m_Backend = VPI_BACKEND_CUDA;

    {
        std::lock_guard<std::mutex> guard(m_SettingsMutex);
        sanitizeSettings(m_Settings);
    }

    m_CreateParams.maxDisparity     = 255;
    m_CreateParams.downscaleFactor  = 1;
    m_CreateParams.includeDiagonals = 1;

    const VPIImageFormat inputFormat = VPI_IMAGE_FORMAT_Y16_ER;

    // Input-side buffers at camera resolution (before rescale)
    CHECK_STATUS(vpiImageCreate(m_CamWidth, m_CamHeight, inputFormat, 0, &m_ImgL_8u));
    CHECK_STATUS(vpiImageCreate(m_CamWidth, m_CamHeight, inputFormat, 0, &m_ImgR_8u));

    // Processing-side buffers at processing resolution (after rescale)
    CHECK_STATUS(vpiImageCreate(m_Width, m_Height, inputFormat, 0, &m_ImgL_Scaled));
    CHECK_STATUS(vpiImageCreate(m_Width, m_Height, inputFormat, 0, &m_ImgR_Scaled));

    // Stereo estimator + output at processing resolution
    CHECK_STATUS(vpiCreateStereoDisparityEstimator(m_Backend, m_Width, m_Height, inputFormat, &m_CreateParams, &m_StereoPayload));
    CHECK_STATUS(vpiImageCreate(m_Width, m_Height, VPI_IMAGE_FORMAT_S16, 0, &m_Disparity));
    CHECK_STATUS(vpiImageCreate(m_Width, m_Height, VPI_IMAGE_FORMAT_U16, 0, &m_ConfidenceMap));

    // Wrapper images at camera resolution (input frames are camera-sized)
    cv::Mat cvImageLeft (m_CamHeight, m_CamWidth, CV_8UC3);
    cv::Mat cvImageRight(m_CamHeight, m_CamWidth, CV_8UC3);
    CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvImageLeft,  VPI_IMAGE_FORMAT_BGR8, VPI_BACKEND_CUDA, &m_ImgL));
    CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvImageRight, VPI_IMAGE_FORMAT_BGR8, VPI_BACKEND_CUDA, &m_ImgR));

    logger->log(Logger::LOG_LVL_INFO, "VideoStereo module initialized (%dx%d)\r\n", m_Width, m_Height);
    return 0;
}


void VideoStereo::setSettings(const Settings& settings) {
    std::lock_guard<std::mutex> guard(m_SettingsMutex);
    m_Settings = settings;
    sanitizeSettings(m_Settings);
}


VideoStereo::Settings VideoStereo::getSettings() {
    std::lock_guard<std::mutex> guard(m_SettingsMutex);
    return m_Settings;
}


int VideoStereo::process(const cv::Mat& rectL,
                         const cv::Mat& rectR,
                         cv::Mat& disparityFrame,
                         cv::Mat& pointCloudMat,
                         cv::Matx44d& Q) {
    if (rectL.empty() || rectR.empty()) {
        return -1;
    }

    int ret = -1;
    cv::Mat cvDisparity, cvDisparityu8, cvDisparityf32, cvDisparityColor, cvPointCloudMat;
    const double sx = static_cast<double>(m_Width) / static_cast<double>(m_CamWidth);
    const double sy = static_cast<double>(m_Height) / static_cast<double>(m_CamHeight);

    cv::cuda::Stream stream;
    cv::cuda::GpuMat d_disp32f, d_xyz;
    cv::Mat zMap(m_Height, m_Width, CV_32FC(6));
    cv::Mat zMapSmooth(m_Height, m_Width, CV_32FC(6));

    Q(0, 3) *= sx;
    Q(1, 3) *= sy;
    Q(2, 3) *= sx;
    Q(3, 3) *= sx; 

    cv::Mat leftBgr, rightBgr;
    if (rectL.channels() == 4) cv::cvtColor(rectL, leftBgr,  cv::COLOR_BGRA2BGR);
    else                       leftBgr  = rectL;
    if (rectR.channels() == 4) cv::cvtColor(rectR, rightBgr, cv::COLOR_BGRA2BGR);
    else                       rightBgr = rectR;

    CHECK_STATUS(vpiImageSetWrappedOpenCVMat(m_ImgL, leftBgr));
    CHECK_STATUS(vpiImageSetWrappedOpenCVMat(m_ImgR, rightBgr));

    // Convert to uint8
    CHECK_STATUS(vpiSubmitConvertImageFormat(m_VpiStream, VPI_BACKEND_CUDA, m_ImgL, m_ImgL_8u, &m_ConvParams));
    CHECK_STATUS(vpiSubmitConvertImageFormat(m_VpiStream, VPI_BACKEND_CUDA, m_ImgR, m_ImgR_8u, &m_ConvParams));

    // Rescale
    CHECK_STATUS(vpiSubmitRescale(m_VpiStream, VPI_BACKEND_CUDA, m_ImgL_8u, m_ImgL_Scaled, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));
    CHECK_STATUS(vpiSubmitRescale(m_VpiStream, VPI_BACKEND_CUDA, m_ImgR_8u, m_ImgR_Scaled, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0));

    // Disparity
    Settings s;
    {
        std::lock_guard<std::mutex> guard(m_SettingsMutex);
        sanitizeSettings(m_Settings);
        s = m_Settings;

        m_StereoParams.maxDisparity        = s.maxDisparity;
        m_StereoParams.confidenceThreshold = s.confidenceThreshold;
        m_StereoParams.p1                  = s.p1;
        m_StereoParams.p2                  = s.p2;
    }
    CHECK_STATUS(vpiSubmitStereoDisparityEstimator(m_VpiStream, VPI_BACKEND_CUDA, m_StereoPayload,
                                                   m_ImgL_Scaled, m_ImgR_Scaled, m_Disparity,
                                                   m_ConfidenceMap, &m_StereoParams));

    // Sync
    CHECK_STATUS(vpiStreamSync(m_VpiStream));

    // Lock disparity
    VPIImageData data;
    CHECK_STATUS(vpiImageLockData(m_Disparity, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data));

    // vpi image -> cv::Mat
    CHECK_STATUS(vpiImageDataExportOpenCVMat(data, &cvDisparity));

    // Q10.5 -> float
    cvDisparity.convertTo(cvDisparityu8, CV_8UC1, 255.0 / (32 * m_CreateParams.maxDisparity), 0);
    applyColorMap(cvDisparityu8, cvDisparityColor, cv::COLORMAP_JET);

    disparityFrame = cvDisparityColor;

    // Convert disparity to float mat
    cvDisparity.convertTo(cvDisparityf32, CV_32F, 1.0 / 32.0, 0);

    // Unlock disparity BEFORE any further VPI/CUDA work on m_Disparity
    CHECK_STATUS(vpiImageUnlock(m_Disparity));

    cv::Mat Q32;
    cv::Mat(Q).convertTo(Q32, CV_32F);

    // Reproject to 3D
    d_disp32f.upload(cvDisparityf32, stream);
    d_xyz.create(d_disp32f.size(), CV_32FC3);
    cv::cuda::reprojectImageTo3D(d_disp32f, d_xyz, Q32, 3, stream);
    d_xyz.download(cvPointCloudMat, stream);
    stream.waitForCompletion();

    {
        std::vector<cv::Mat> channels(3);
        cv::split(cvPointCloudMat, channels);
        cv::Mat X = channels[0], Y = channels[1], Z = channels[2];

        // Finite check (NaN != NaN)
        cv::Mat finite_mask = (X == X) & (Y == Y) & (Z == Z);

        // Filter out Inf
        cv::Mat abs_x, abs_y, abs_z;
        cv::absdiff(X, cv::Scalar(0), abs_x);
        cv::absdiff(Y, cv::Scalar(0), abs_y);
        cv::absdiff(Z, cv::Scalar(0), abs_z);
        finite_mask &= (abs_x < 1e10f) & (abs_y < 1e10f) & (abs_z < 1e10f);

        // Range filter
        cv::Mat pc_mask = finite_mask
            & (Z > s.zMin)
            & (Z < s.zMax)
            & (abs_x < 12.0f)
            & (abs_y < 8.0f);

        // Apply mask — zero out invalid points
        cv::Mat filtered = cv::Mat::zeros(cvPointCloudMat.size(), cvPointCloudMat.type());
        cvPointCloudMat.copyTo(filtered, pc_mask);

        // Fallback
        if (cv::countNonZero(pc_mask) == 0) {
            cvPointCloudMat.copyTo(filtered, finite_mask);
        }

        cvPointCloudMat = filtered;
    }

    // Resize the left colour image to match the point-cloud resolution
    cv::Mat colorResized;
    cv::resize(leftBgr, colorResized, cv::Size(m_Width, m_Height));

    // Fill in the XYZ coordinates and color
    ret = cuda::pointCloudToColor(cvPointCloudMat, colorResized, zMap);
    if (ret < 0) {
        printf("Failed convert point cloud\n");
    }

    ret = cuda::smoothPointCloud(zMap, zMapSmooth, s.depthThreshold, s.minAgreeingPixels, s.colorThreshold);
    if (ret < 0) {
        printf("Failed to smooth pointcloud\n");
    }

    pointCloudMat = zMapSmooth;
    return 0;
}
}
