#include "include/cudaVision.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <cuda_runtime.h>


/**
 * @brief Merges a CV_32FC3 point cloud (XYZ) with a CV_8UC3 color frame (BGR)
 *        into a CV_32FC(6) output (X, Y, Z, B, G, R as floats).
 *
 * @param pointCloud      Pointer to XYZ data  (3 floats per pixel)
 * @param pcStep          Point-cloud row stride in bytes (GpuMat::step)
 * @param colorFrame      Pointer to BGR data  (3 unsigned bytes per pixel)
 * @param cfStep          Color-frame row stride in bytes (GpuMat::step)
 * @param dstMat          Pointer to output    (6 floats per pixel)
 * @param dstStep         Output row stride in bytes (GpuMat::step)
 * @param rows            Image height
 * @param cols            Image width
 */
__global__ void pointCloudToColorKernel(const float* __restrict__ pointCloud,
                                        const size_t pcStep,
                                        const unsigned char* __restrict__ colorFrame,
                                        const size_t cfStep,
                                        float* __restrict__ dstMat,
                                        const size_t dstStep,
                                        const int rows,
                                        const int cols) {
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = blockIdx.y * blockDim.y + threadIdx.y;

    if (row >= rows || col >= cols) {
        return;
    }

    // Use byte-based row offsets to account for GpuMat padding
    const float*         xyzRow = reinterpret_cast<const float*>(
                                      reinterpret_cast<const unsigned char*>(pointCloud) + row * pcStep);
    const unsigned char* bgrRow = colorFrame + row * cfStep;
    float*               dstRow = reinterpret_cast<float*>(
                                      reinterpret_cast<unsigned char*>(dstMat) + row * dstStep);

    // Source pointers (column offset within the row)
    const float*         xyz = xyzRow + col * 3;   // CV_32FC3: 3 floats
    const unsigned char* bgr = bgrRow + col * 3;   // CV_8UC3:  3 bytes

    // Destination pointer
    float* dst = dstRow + col * 6;                 // CV_32FC(6): 6 floats

    // Channels 0-2: XYZ from point cloud
    dst[0] = xyz[0];
    dst[1] = xyz[1];
    dst[2] = xyz[2];

    // Channels 3-5: BGR colour converted to float [0..255]
    dst[3] = static_cast<float>(bgr[0]);
    dst[4] = static_cast<float>(bgr[1]);
    dst[5] = static_cast<float>(bgr[2]);
}


int cuda::pointCloudToColor(cv::Mat& pointCloud, cv::Mat& colorFrame, cv::Mat& colorPointCloud) {
    if (pointCloud.type() != CV_32FC3 || colorFrame.type() != CV_8UC3 || colorPointCloud.type() != CV_32FC(6)) {
        return -1; // Unsupported types
    }

    CV_Assert(pointCloud.size() == colorFrame.size());
    CV_Assert(pointCloud.size() == colorPointCloud.size());

    dim3 block(16, 16);
    dim3 grid(
        (pointCloud.cols + block.x - 1) / block.x,
        (pointCloud.rows + block.y - 1) / block.y
    );

    // Upload to GPU
    cv::cuda::GpuMat d_pointCloud(pointCloud);
    cv::cuda::GpuMat d_colorFrame(colorFrame);
    cv::cuda::GpuMat d_colorPointCloud(colorPointCloud);

    pointCloudToColorKernel<<<grid, block>>>(
        d_pointCloud.ptr<float>(),
        d_pointCloud.step,
        d_colorFrame.ptr<unsigned char>(),
        d_colorFrame.step,
        d_colorPointCloud.ptr<float>(),
        d_colorPointCloud.step,
        pointCloud.rows,
        pointCloud.cols
    );

    // Wait for kernel to finish, then download result
    cudaDeviceSynchronize();
    d_colorPointCloud.download(colorPointCloud);

    return 0;
}