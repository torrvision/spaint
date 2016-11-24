/**
 * spaint: RGBDPatchFeatureCalculator_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.h"
#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace spaint
{
__global__ void ck_compute_colour_feature(Keypoint3DColour *keypoints,
    RGBDPatchDescriptor *features, const Vector4u *rgb, const float *depth,
    const Vector4i *offsetsRgb, const uchar *channelsRgb, Vector2i imgSize,
    Vector2i outSize, Vector4f intrinsics, Matrix4f cameraPose,
    uint32_t featureStep, bool normalize)
{
  const Vector2i xyOut(threadIdx.x + blockIdx.x * blockDim.x,
      threadIdx.y + blockIdx.y * blockDim.y);

  if (xyOut.x >= outSize.x || xyOut.y >= outSize.y)
    return;

  const Vector2i xyIn(xyOut.x * featureStep, xyOut.y * featureStep);

  compute_colour_patch_feature(keypoints, features, rgb, depth, offsetsRgb,
      channelsRgb, imgSize, outSize, intrinsics, cameraPose, normalize, xyIn,
      xyOut);
}

__global__ void ck_compute_depth_feature(Keypoint3DColour *keypoints,
    RGBDPatchDescriptor *features, const float *depth,
    const Vector4i *offsetsDepth, Vector2i imgSize, Vector2i outSize,
    uint32_t featureStep, bool normalize)
{
  const Vector2i xyOut(threadIdx.x + blockIdx.x * blockDim.x,
      threadIdx.y + blockIdx.y * blockDim.y);

  if (xyOut.x >= outSize.x || xyOut.y >= outSize.y)
    return;

  const Vector2i xyIn(xyOut.x * featureStep, xyOut.y * featureStep);

  compute_depth_patch_feature(keypoints, features, depth, offsetsDepth, imgSize,
      outSize, normalize, xyIn, xyOut);
}

RGBDPatchFeatureCalculator_CUDA::RGBDPatchFeatureCalculator_CUDA()
{
  m_offsetsRgb->UpdateDeviceFromHost();
  m_channelsRgb->UpdateDeviceFromHost();
  m_offsetsDepth->UpdateDeviceFromHost();
}

void RGBDPatchFeatureCalculator_CUDA::compute_feature(
    const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
    const Vector4f &intrinsics, Keypoint3DColourImage *keypointsImage,
    RGBDPatchDescriptorImage *featuresImage, const Matrix4f &cameraPose) const
{
  const Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CUDA);
  const float *depth = depthImage->GetData(MEMORYDEVICE_CUDA);

  const Vector4i *offsetsRgb = m_offsetsRgb->GetData(MEMORYDEVICE_CUDA);
  const uchar *channelsRgb = m_channelsRgb->GetData(MEMORYDEVICE_CUDA);
  const Vector4i *offsetsDepth = m_offsetsDepth->GetData(MEMORYDEVICE_CUDA);

  Vector2i inDims = rgbImage->noDims;
  Vector2i outDims(rgbImage->noDims.x / m_featureStep,
      rgbImage->noDims.y / m_featureStep);

  keypointsImage->ChangeDims(outDims);
  featuresImage->ChangeDims(outDims);
  Keypoint3DColour *keypoints = keypointsImage->GetData(MEMORYDEVICE_CUDA);
  RGBDPatchDescriptor *features = featuresImage->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(32, 32);
  dim3 gridSize((outDims.x + blockSize.x - 1) / blockSize.x,
      (outDims.y + blockSize.y - 1) / blockSize.y);

  // The colour feature calculator also computed the 3D pose of the point
  // and fills the colour in the feature struct. Intrinsics are only needed here
  ck_compute_colour_feature<<<gridSize, blockSize>>>(keypoints, features, rgb, depth, offsetsRgb, channelsRgb,
      inDims, outDims, intrinsics, cameraPose, m_featureStep, m_normalizeRgb);
  ORcudaKernelCheck;

  ck_compute_depth_feature<<<gridSize, blockSize>>>(keypoints, features, depth, offsetsDepth,
      inDims, outDims, m_featureStep, m_normalizeDepth);
  ORcudaKernelCheck;
}

}
