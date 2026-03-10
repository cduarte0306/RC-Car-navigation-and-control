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
 
    /**
     * @brief Reject speckle noise from the point cloud.  Pixels whose depth
     *        does not agree with enough neighbours are replaced by the average
     *        of colour-similar neighbours (same surface).  If no colour-similar
     *        neighbours exist, the pixel is zeroed out.
     *
     * @param pointCloud         Reference to input 6-channel point cloud
     * @param smoothedPointCloud Reference to output filtered point cloud
     * @param depthThreshold     Max Z difference (metres) to count a neighbour as depth-agreeing
     * @param minAgreeing        Minimum depth-agreeing neighbours to keep the pixel unchanged
     * @param colorThreshold     Max Euclidean RGB distance to count a neighbour as colour-similar
     * @return int Error code
     */
    int smoothPointCloud(cv::Mat& pointCloud, cv::Mat& smoothedPointCloud,
                         float depthThreshold = 0.01f, int minAgreeing = 20,
                         float colorThreshold = 30.0f);
};

#pragma endregion
