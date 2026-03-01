#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>

namespace cuda {

    /**
     * @brief Returns a 6-channel point cloud with color
     * 
     * @param pointCloud Reference to 3 channel point cloud
     * @param colorFrame Reference to stereo color frame
     * @return int Error code
     */
    int pointCloudToColor(cv::Mat& pointCloud, cv::Mat& colorFrame, cv::Mat& colorPointCloud);
    
};

#pragma endregion
