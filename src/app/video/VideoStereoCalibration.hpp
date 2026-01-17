#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace Vision {
class VideoStereoCalib {
public:
    VideoStereoCalib();
    VideoStereoCalib(const VideoStereoCalib&) = delete;
    VideoStereoCalib& operator=(const VideoStereoCalib&) = delete;
    ~VideoStereoCalib() = default;

    int storeCalibrationProfile(const char* profileName);

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
};
}

#pragma endregion
