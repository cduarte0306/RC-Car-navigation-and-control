#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>


namespace Vision {
class VideoStereoCalib {
public:
    VideoStereoCalib();
    VideoStereoCalib(const VideoStereoCalib&) = delete;
    VideoStereoCalib& operator=(const VideoStereoCalib&) = delete;
    ~VideoStereoCalib() = default;

    int storeCalibrationProfile(const char* profileName);

    void resetCalibrationSession();
    std::size_t numCollectedSamples() const;
    bool isCalibrated() const;
    double lastRmsError() const;

    /**
     * @brief Configure calibration settings from a JSON string received from the remote UI.
     *
     * Expected fields (others ignored):
     * - target.pattern.cols / rows
     * - target.square_size.value / units ("mm" or "m")
     * - capture.required_samples
     * - output_view.show_overlays
     * - profile_name
     *
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
        std::string profileName{"default"};
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

    std::vector<std::vector<cv::Point2f>> m_ImagePointsL;
    std::vector<std::vector<cv::Point2f>> m_ImagePointsR;
    std::vector<std::vector<cv::Point3f>> m_ObjectPoints;
    cv::Size m_ImageSize;
    bool m_Calibrated{false};
    double m_LastRmsError{0.0};
};
}

#pragma endregion
