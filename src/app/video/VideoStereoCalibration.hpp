#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

#include <nlohmann/json.hpp>

namespace Vision {
class VideoStereoCalib {
public:
    VideoStereoCalib();
    VideoStereoCalib(const VideoStereoCalib&) = delete;
    VideoStereoCalib& operator=(const VideoStereoCalib&) = delete;
    ~VideoStereoCalib() = default;

    int storeCalibrationProfile();

    void resetCalibrationSession();
    std::size_t numCollectedSamples() const;
    bool isCalibrated() const;
    double lastRmsError() const;

    /**
     * @brief Convert a disparity value (pixels) to depth (meters).
     *
     * Requires a calibrated/loaded rectification profile with valid P2.
     * Disparity must be > 0.
     */
    std::optional<double> depthMetersFromDisparity(double disparityPx) const;

    /**
     * @brief Get rectified focal length in pixels (fx).
     *
     * Uses P1(0,0) from the loaded/computed rectification profile.
     */
    double focalLengthPx() const;

    cv::Size imageSize() const;
    bool hasRectification() const;

    cv::Matx33d leftK() const;
    cv::Matx33d rightK() const;

    cv::Matx33d stereoR() const;
    cv::Vec3d stereoT() const;
    double baselineMeters() const;

    cv::Matx33d rectifyR1() const;
    cv::Matx33d rectifyR2() const;
    cv::Matx34d projectionP1() const;
    cv::Matx34d projectionP2() const;

    /**
     * @brief Get the 4x4 reprojection (Q) matrix from the loaded/computed rectification profile.
     */
    cv::Matx44d reprojectionQ() const;

    /**
     * @brief Configure calibration settings from a JSON string received from the remote UI.
     *
     * Expected fields (others ignored):
     * - target.pattern.cols / rows
     * - target.square_size.value / units ("mm" or "m")
     * - capture.required_samples
     * - output_view.show_overlays
     * @return int 0 on success, negative on failure.
     */
    int configureFromJson(const std::string& jsonStr);

    /**
     * @brief Run one calibration step using the latest configured settings (from configureFromJson()).
     *
     * This performs live detection every call, collects samples when detected, and runs calibration
     * once the configured sample count is reached.
     */
    int DoCalibration(cv::Mat& leftBgr, cv::Mat& rightBgr);

    /**
     * @brief Perform video calibration
     * 
     * @param frameL Left stereo frame
     * @param frameR Right stereo frame
     * @return int Error code
     */
    int DetectChessBoard(cv::Mat& leftBgr, cv::Mat& rightBgr, cv::Size boardSize);

    /**
     * @brief Calibrates stereo camera
     * 
     * @param leftBgr Left frame
     * @param rightBgr Right frame
     * @return int Error code
     */
    int DoCalibration(cv::Mat& leftBgr,
                      cv::Mat& rightBgr,
                      cv::Size boardSize,
                      double squareSize,
                      std::size_t targetSamples = 25,
                      bool drawOverlay = true);

    /**
     * @brief Rectify stereo frames
     * 
     * @param leftIn Input left frame
     * @param rightIn Input right frame
     * @param leftRect Output rectified left frame
     * @param rightRect Output rectified right frame
     */
    void rectify(const cv::Mat& leftIn, const cv::Mat& rightIn,
                 cv::Mat& leftRect, cv::Mat& rightRect) const;

    /**
     * @brief Get the current calibration as a JSON object
     * @return nlohmann::json Current calibration
     */
    nlohmann::json& getCurrentCalibrationStats();

private:
    /**
     * @brief Load the configured calibration profile
     * 
     * @param path Path to the calibration profile
     * @return int Error code
     */
    int loadCalibrationProfile(const char* path);

    struct CalibrationProfileSettings {
        static constexpr size_t KSize = 9;
        static constexpr size_t DSize = 5;
        static constexpr size_t RSize = 9;
        static constexpr size_t TSize = 3;
        static constexpr size_t PSize = 12;
        static constexpr size_t QSize = 16;

        uint32_t version = 1;
        std::string model = "pinhole";

        uint32_t imageWidth = 0;
        uint32_t imageHeight = 0;

        std::array<double, KSize> K_left{0, 0, 0,
                                         0, 0, 0,
                                         0, 0, 1};
        std::array<double, DSize> D_left{0, 0, 0, 0, 0};

        std::array<double, KSize> K_right{0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 1};
        std::array<double, DSize> D_right{0, 0, 0, 0, 0};

        std::array<double, RSize> R{1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1};
        std::array<double, TSize> T{0, 0, 0};

        std::array<double, RSize> R1{1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1};
        std::array<double, RSize> R2{1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1};
        std::array<double, PSize> P1{0, 0, 0, 0,
                                     0, 0, 0, 0,
                                     0, 0, 1, 0};
        std::array<double, PSize> P2{0, 0, 0, 0,
                                     0, 0, 0, 0,
                                     0, 0, 1, 0};
        std::array<double, QSize> Q{1, 0, 0, 0,
                                    0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1};

        bool hasImageSize() const {
            return imageWidth > 0 && imageHeight > 0;
        }
    };

    /**
     * @brief Video calibration default path
     * 
     */
    const char* calibStoragePath = "/data/calibration-data/stereo-calibration/stereo-profile.json";

    /**
     * @brief Local profile settings
     * 
     */
    CalibrationProfileSettings profileSettings_{};

    struct CalibrationRequest {
        cv::Size boardSize{9, 6};
        double squareSizeMeters{0.0};
        std::size_t requiredSamples{25};
        bool showOverlays{true};
    } calibReq_{};

    static constexpr const char* DefaultProfileJson = R"json(
{
  "version": 1,
  "model": "pinhole",
  "image_size": { "width": 0, "height": 0 },
  "left": {
    "K": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
    "D": [0, 0, 0, 0, 0]
  },
  "right": {
    "K": [[0, 0, 0], [0, 0, 0], [0, 0, 1]],
    "D": [0, 0, 0, 0, 0]
  },
  "stereo": {
    "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    "T": [0, 0, 0]
  },
  "rectify": {
    "R1": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    "R2": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    "P1": [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0]],
    "P2": [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0]],
    "Q": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
  }
}
)json";

    cv::Mat map1L, map2L, map1R, map2R;
    cv::Size m_RectifySize{};
    bool m_HaveRectifyMaps{false};

    std::vector<std::vector<cv::Point2f>> m_ImagePointsL;
    std::vector<std::vector<cv::Point2f>> m_ImagePointsR;
    std::vector<std::vector<cv::Point3f>> m_ObjectPoints;
    cv::Size m_ImageSize;
    bool m_Calibrated{false};
    double m_LastRmsError{0.0};

    nlohmann::json m_CalibrationStatsBank;

    bool buildRectifyMaps_();
};
}

#pragma endregion
