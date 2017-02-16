/**
 * spaint: RGBDPatchFeatureCalculator_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.h"
#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace spaint
{
//#################### CUDA KERNELS ####################
__global__ void ck_compute_colour_feature(Keypoint3DColour *keypoints,
    RGBDPatchDescriptor *features, const Vector4u *rgb, const float *depth,
    const Vector4i *offsetsRgb, const uchar *channelsRgb, Vector2i imgSize,
    Vector2i outSize, uint32_t featureStep, bool normalize, uint32_t featuresCount,
    uint32_t outputFeaturesOffset)
{
  // Coordinates of the output keypoint/descriptor pair.
  const Vector2i xyOut(threadIdx.x + blockIdx.x * blockDim.x,
      threadIdx.y + blockIdx.y * blockDim.y);

  if (xyOut.x >= outSize.x || xyOut.y >= outSize.y)
    return;

  // Coordinates of the pixel used as center of the feature.
  const Vector2i xyIn(xyOut.x * featureStep, xyOut.y * featureStep);

  compute_colour_patch_feature(keypoints, features, rgb, depth, offsetsRgb,
      channelsRgb, imgSize, outSize, normalize, xyIn, xyOut, featuresCount, outputFeaturesOffset);
}

__global__ void ck_compute_depth_feature(Keypoint3DColour *keypoints,
    RGBDPatchDescriptor *features, const float *depth,
    const Vector4i *offsetsDepth, Vector2i imgSize, Vector2i outSize,
    Vector4f intrinsics, Matrix4f cameraPose, uint32_t featureStep,
    bool normalize, uint32_t featuresCount, uint32_t outputFeaturesOffset)
{
  // Coordinates of the output keypoint/descriptor pair.
  const Vector2i xyOut(threadIdx.x + blockIdx.x * blockDim.x,
      threadIdx.y + blockIdx.y * blockDim.y);

  if (xyOut.x >= outSize.x || xyOut.y >= outSize.y)
    return;

  // Coordinates of the pixel used as center of the feature.
  const Vector2i xyIn(xyOut.x * featureStep, xyOut.y * featureStep);

  compute_depth_patch_feature(keypoints, features, depth, offsetsDepth, imgSize,
      outSize, intrinsics, cameraPose, normalize, xyIn, xyOut, featuresCount, outputFeaturesOffset);
}

//#################### CONSTRUCTORS ####################

RGBDPatchFeatureCalculator_CUDA::RGBDPatchFeatureCalculator_CUDA()
{
  // Update the offset for use on the GPU.
  m_offsetsRgb->UpdateDeviceFromHost();
  m_channelsRgb->UpdateDeviceFromHost();
  m_offsetsDepth->UpdateDeviceFromHost();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void RGBDPatchFeatureCalculator_CUDA::compute_feature(
    const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
    const Vector4f &intrinsics, Keypoint3DColourImage *keypointsImage,
    RGBDPatchDescriptorImage *featuresImage, const Matrix4f &cameraPose) const
{
  const float *depth = depthImage->GetData(MEMORYDEVICE_CUDA);
  const Vector4i *offsetsDepth = m_offsetsDepth->GetData(MEMORYDEVICE_CUDA);

  const Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CUDA);
  const Vector4i *offsetsRgb = m_offsetsRgb->GetData(MEMORYDEVICE_CUDA);
  const uchar *channelsRgb = m_channelsRgb->GetData(MEMORYDEVICE_CUDA);

  Vector2i inDims = depthImage->noDims;
  // The output images have one pixel per each element of the sampling grid.
  Vector2i outDims(depthImage->noDims.x / m_featureStep,
      depthImage->noDims.y / m_featureStep);

  // Resize images appropriately. Will always be a NOP except the first time.
  keypointsImage->ChangeDims(outDims);
  featuresImage->ChangeDims(outDims);

  Keypoint3DColour *keypoints = keypointsImage->GetData(MEMORYDEVICE_CUDA);
  RGBDPatchDescriptor *features = featuresImage->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(32, 32);
  dim3 gridSize((outDims.x + blockSize.x - 1) / blockSize.x,
      (outDims.y + blockSize.y - 1) / blockSize.y);

  // The depth feature calculator also computes the 3D pose and colour of the keypoint
  // Camera intrinsics are only needed here
  ck_compute_depth_feature<<<gridSize, blockSize>>>(keypoints, features, depth, offsetsDepth,
      inDims, outDims, intrinsics, cameraPose, m_featureStep, m_normalizeDepth, m_countDepthFeatures,
      m_offsetDepthFeatures);
  ORcudaKernelCheck;

  if(rgb)
  {
    ck_compute_colour_feature<<<gridSize, blockSize>>>(keypoints, features, rgb, depth, offsetsRgb, channelsRgb,
        inDims, outDims, m_featureStep, m_normalizeRgb, m_countRgbFeatures, m_offsetRgbFeatures);
    ORcudaKernelCheck;
  }
}

}
