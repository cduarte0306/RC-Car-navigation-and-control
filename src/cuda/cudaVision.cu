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
    dst[3] = static_cast<float>(bgr[2]);
    dst[4] = static_cast<float>(bgr[1]);
    dst[5] = static_cast<float>(bgr[0]);
}


/**
 * @brief Surface-aware speckle filter.
 *
 *  Every valid pixel finds colour-similar neighbours in a 7×7 window and
 *  replaces its XYZ with the average of those neighbours.  This normalises
 *  depth across same-colour surfaces, eliminating inward rays.  The pixel
 *  keeps its own colour so the visual appearance is unchanged.
 *
 *  Pixels that have NO colour-similar neighbours AND fail the depth-consensus
 *  check (fewer than minAgreeing depth-agreeing neighbours) are considered
 *  true speckles and zeroed out entirely.
 */
__global__ void speckleRejectionKernel(const float* __restrict__ pointCloud,
                                       const size_t pcStep,
                                       float* __restrict__ filteredPointCloud,
                                       const size_t fpcStep,
                                       const int rows,
                                       const int cols,
                                       const float depthThreshold,
                                       const int   minAgreeing,
                                       const float colorThreshold) {
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = blockIdx.y * blockDim.y + threadIdx.y;

    if (row >= rows || col >= cols) {
        return;
    }

    const int CH = 6;   // CV_32FC(6): X, Y, Z, R, G, B
    const int halfN = 3; // 7×7 window

    const float* srcRow = reinterpret_cast<const float*>(
                              reinterpret_cast<const unsigned char*>(pointCloud) + row * pcStep);
    float*       dstRow = reinterpret_cast<float*>(
                              reinterpret_cast<unsigned char*>(filteredPointCloud) + row * fpcStep);

    const float* center = srcRow + col * CH;
    float        centerZ = center[2];

    float* dst = dstRow + col * CH;

    // This function should fill in pixels where point cloud is invalid based on neihboring values. We need 
    // to iterate throuh every pixel that is 0, we 
    

    // If center pixel is already invalid, keep it zeroed
    if (centerZ <= 0.0f) {
        dst[0] = 0.0f; dst[1] = 0.0f; dst[2] = 0.0f;
        dst[3] = 0.0f; dst[4] = 0.0f; dst[5] = 0.0f;
        return;
    }

    float cR = center[3], cG = center[4], cB = center[5];

    // Single pass: gather colour-similar neighbours AND count depth-agreeing ones
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    int colorCount  = 0;
    int depthAgreeing = 0;

    for (int dy = -halfN; dy <= halfN; ++dy) {
        int r = row + dy;
        if (r < 0 || r >= rows) continue;

        const float* nRow = reinterpret_cast<const float*>(
                                reinterpret_cast<const unsigned char*>(pointCloud) + r * pcStep);

        for (int dx = -halfN; dx <= halfN; ++dx) {
            if (dx == 0 && dy == 0) continue; // skip self
            int c = col + dx;
            if (c < 0 || c >= cols) continue;

            const float* nb = nRow + c * CH;
            float nZ = nb[2];
            if (nZ <= 0.0f) continue; // skip invalid

            // Depth consensus
            if (fabsf(nZ - centerZ) <= depthThreshold) {
                depthAgreeing++;
            }

            // Colour similarity — accumulate XYZ for surface averaging
            float dR = nb[3] - cR;
            float dG = nb[4] - cG;
            float dB = nb[5] - cB;
            float colorDist = sqrtf(dR * dR + dG * dG + dB * dB);

            if (colorDist <= colorThreshold) {
                sumX += nb[0]; sumY += nb[1]; sumZ += nb[2];
                colorCount++;
            }
        }
    }

    if (colorCount > 0) {
        // Snap XYZ to the average of colour-similar neighbours (same surface)
        // Keep the pixel's own colour unchanged
        float inv = 1.0f / colorCount;
        dst[0] = sumX * inv;
        dst[1] = sumY * inv;
        dst[2] = sumZ * inv;
        dst[3] = cR;
        dst[4] = cG;
        dst[5] = cB;
    } else if (depthAgreeing >= minAgreeing) {
        // No colour matches but depth is consistent — keep as-is
        dst[0] = center[0]; dst[1] = center[1]; dst[2] = center[2];
        dst[3] = cR;        dst[4] = cG;        dst[5] = cB;
    } else {
        // True speckle: no colour match AND depth is an outlier — remove
        dst[0] = 0.0f; dst[1] = 0.0f; dst[2] = 0.0f;
        dst[3] = 0.0f; dst[4] = 0.0f; dst[5] = 0.0f;
    }
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


int cuda::smoothPointCloud(cv::Mat& pointCloud, cv::Mat& smoothedPointCloud,
                           float depthThreshold, int minAgreeing,
                           float colorThreshold) {
    if (pointCloud.type() != CV_32FC(6) || smoothedPointCloud.type() != CV_32FC(6)) {
        return -1; // Unsupported types
    }

    if (minAgreeing = 0) {
        smoothedPointCloud = pointCloud;
        return 0;
    }

    CV_Assert(pointCloud.size() == smoothedPointCloud.size());

    dim3 block(16, 16);
    dim3 grid(
        (pointCloud.cols + block.x - 1) / block.x,
        (pointCloud.rows + block.y - 1) / block.y
    );

    // Upload to GPU
    cv::cuda::GpuMat d_pointCloud(pointCloud);
    cv::cuda::GpuMat d_smoothedPointCloud(smoothedPointCloud);

    speckleRejectionKernel<<<grid, block>>>(
        d_pointCloud.ptr<float>(),
        d_pointCloud.step,
        d_smoothedPointCloud.ptr<float>(),
        d_smoothedPointCloud.step,
        pointCloud.rows,
        pointCloud.cols,
        depthThreshold,
        minAgreeing,
        colorThreshold
    );

    // Wait for kernel to finish, then download result
    cudaDeviceSynchronize();
    d_smoothedPointCloud.download(smoothedPointCloud);

    return 0;
}