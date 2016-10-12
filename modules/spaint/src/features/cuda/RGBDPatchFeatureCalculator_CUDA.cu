/**
 * spaint: RGBDPatchFeatureCalculator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.h"
#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace spaint
{
__global__ void ck_compute_colour_feature(RGBDPatchFeature *features,
    const Vector4u *rgb, const float *depth, const Vector4i *offsets_rgb,
    const uchar *channels_rgb, Vector2i img_size, int feature_step, bool normalize)
{
  const int x = (threadIdx.x + blockIdx.x * blockDim.x) * feature_step;
  const int y = (threadIdx.y + blockIdx.y * blockDim.y) * feature_step;

  if (x >= img_size.x || y >= img_size.y)
    return;

  compute_colour_patch_feature(features, rgb, depth, offsets_rgb, channels_rgb,
      img_size, normalize, x, y);
}

__global__ void ck_compute_depth_feature(RGBDPatchFeature *features,
    const float *depth, const Vector4i *offsets_depth,
    Vector2i img_size, int feature_step, bool normalize)
{
  const int x = (threadIdx.x + blockIdx.x * blockDim.x) * feature_step;
  const int y = (threadIdx.y + blockIdx.y * blockDim.y) * feature_step;

  if (x >= img_size.x || y >= img_size.y)
    return;

  compute_depth_patch_feature(features, depth, offsets_depth, img_size, normalize, x, y);
}

RGBDPatchFeatureCalculator_CUDA::RGBDPatchFeatureCalculator_CUDA()
{
  m_offsetsRgb->UpdateDeviceFromHost();
  m_channelsRgb->UpdateDeviceFromHost();
  m_offsetsDepth->UpdateDeviceFromHost();
}

void RGBDPatchFeatureCalculator_CUDA::ComputeFeature(
    const ITMUChar4Image_CPtr &rgb_image, const ITMFloatImage_CPtr &depth_image,
    boost::shared_ptr<ORUtils::Image<RGBDPatchFeature> > &features_image) const
{
  const Vector4u *rgb = rgb_image->GetData(MEMORYDEVICE_CUDA);
  const float *depth = depth_image->GetData(MEMORYDEVICE_CUDA);

  const Vector4i *offsets_rgb = m_offsetsRgb->GetData(MEMORYDEVICE_CUDA);
  const uchar *channels_rgb = m_channelsRgb->GetData(MEMORYDEVICE_CUDA);
  const Vector4i *offsets_depth = m_offsetsDepth->GetData(MEMORYDEVICE_CUDA);

  if(features_image->noDims != rgb_image->noDims) // Just for the call to Clear()
  {
    features_image->ChangeDims(rgb_image->noDims);
    features_image->Clear();
  }

  RGBDPatchFeature *features = features_image->GetData(MEMORYDEVICE_CUDA);

  Vector2i out_dims(rgb_image->noDims.x / m_featureStep, rgb_image->noDims.y / m_featureStep);

  dim3 blockSize(32, 32);
  dim3 gridSize((out_dims.x + blockSize.x - 1) / blockSize.x, (out_dims.y + blockSize.y - 1) / blockSize.y);

  ck_compute_colour_feature<<<gridSize, blockSize>>>(features, rgb, depth, offsets_rgb, channels_rgb,
      rgb_image->noDims, m_featureStep, m_normalizeRgb);
  cudaDeviceSynchronize();

  ck_compute_depth_feature<<<gridSize, blockSize>>>(features, depth, offsets_depth,
      depth_image->noDims, m_featureStep, m_normalizeDepth);
  cudaDeviceSynchronize();
}

}
