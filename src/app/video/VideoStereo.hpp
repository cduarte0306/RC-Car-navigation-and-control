#pragma once

#include <cstdint>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>


namespace Vision {
class VideoStereo {
public:
    struct Settings {
        int numDisparities      = 64;
        int minDisparity        = 0;
        int maxDisparity        = 0;
        int confidenceThreshold = 32767;
        int confidenceType      = VPI_STEREO_CONFIDENCE_ABSOLUTE;
        int vpiQuality          = 1;
        int p1                  = 3;
        int p2                  = 48;
        int p2Alpha             = 0;
        float uniqueness        = -1.0f;
        int numPasses           = 3;
        int uniquenessRatio     = 0;

        float zMin              = 0.0f;
        float zMax              = 10.0f;
        float depthThreshold    = 0.01f;
        int   minAgreeingPixels = 20;
        float colorThreshold    = 30.0f;
    };

    VideoStereo();
    ~VideoStereo();

    int init(int camWidth, int camHeight, int processWidth, int processHeight);

    /**
     * @brief Run VPI stereo disparity estimation on a rectified stereo pair
     * and produce a colorized disparity image and a filtered/smoothed point
     * cloud (CV_32FC(6)).
     */
    int process(const cv::Mat& rectL,
                const cv::Mat& rectR,
                cv::Mat& disparityFrame,
                cv::Mat& pointCloudMat,
                cv::Matx44d& Q);

    void setSettings(const Settings& settings);
    Settings getSettings();

    /**
     * @brief Clamp VPI-specific settings into valid ranges.
     */
    static void sanitizeSettings(Settings& settings);

protected:
    int m_CamWidth  = 0;
    int m_CamHeight = 0;
    int m_Width  = 0;
    int m_Height = 0;
    uint64_t m_Backend = 0;

    std::mutex m_SettingsMutex;
    Settings m_Settings;

    VPIStream  m_VpiStream     = nullptr;
    VPIPayload m_StereoPayload = nullptr;

    VPIImage m_ImgL          = nullptr;
    VPIImage m_ImgR          = nullptr;
    VPIImage m_ImgL_8u       = nullptr;
    VPIImage m_ImgR_8u       = nullptr;
    VPIImage m_ImgL_Scaled   = nullptr;
    VPIImage m_ImgR_Scaled   = nullptr;
    VPIImage m_Disparity     = nullptr;
    VPIImage m_ConfidenceMap = nullptr;

    VPIStereoDisparityEstimatorCreationParams m_CreateParams;
    VPIStereoDisparityEstimatorParams m_StereoParams;
    VPIConvertImageFormatParams m_ConvParams;
};
}
