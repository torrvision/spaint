/**
 * spaint: RGBDPatchFeatureCalculator_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.h"
#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace spaint
{
__global__ void ck_compute_colour_feature(RGBDPatchFeature *features,
    const Vector4u *rgb, const float *depth, const Vector4i *offsets_rgb,
    const uchar *channels_rgb, Vector2i img_size, Vector2i out_size,
    Vector4f intrinsics, Matrix4f cameraPose, int feature_step, bool normalize)
{
  const Vector2i xy_out(threadIdx.x + blockIdx.x * blockDim.x,
      threadIdx.y + blockIdx.y * blockDim.y);

  if (xy_out.x >= out_size.x || xy_out.y >= out_size.y)
    return;

  const Vector2i xy_in(xy_out.x * feature_step, xy_out.y * feature_step);

  compute_colour_patch_feature(features, rgb, depth, offsets_rgb, channels_rgb,
      img_size, out_size, intrinsics, cameraPose, normalize, xy_in, xy_out);
}

__global__ void ck_compute_depth_feature(RGBDPatchFeature *features,
    const float *depth, const Vector4i *offsets_depth, Vector2i img_size,
    Vector2i out_size, int feature_step, bool normalize)
{
  const Vector2i xy_out(threadIdx.x + blockIdx.x * blockDim.x,
      threadIdx.y + blockIdx.y * blockDim.y);

  if (xy_out.x >= out_size.x || xy_out.y >= out_size.y)
    return;

  const Vector2i xy_in(xy_out.x * feature_step, xy_out.y * feature_step);

  compute_depth_patch_feature(features, depth, offsets_depth, img_size,
      out_size, normalize, xy_in, xy_out);
}

RGBDPatchFeatureCalculator_CUDA::RGBDPatchFeatureCalculator_CUDA()
{
  m_offsetsRgb->UpdateDeviceFromHost();
  m_channelsRgb->UpdateDeviceFromHost();
  m_offsetsDepth->UpdateDeviceFromHost();
}

void RGBDPatchFeatureCalculator_CUDA::ComputeFeature(
    const ITMUChar4Image *rgb_image, const ITMFloatImage *depth_image,
    const Vector4f &intrinsics, RGBDPatchFeatureImage *features_image,
    const Matrix4f &cameraPose) const
{
  const Vector4u *rgb = rgb_image->GetData(MEMORYDEVICE_CUDA);
  const float *depth = depth_image->GetData(MEMORYDEVICE_CUDA);

  const Vector4i *offsets_rgb = m_offsetsRgb->GetData(MEMORYDEVICE_CUDA);
  const uchar *channels_rgb = m_channelsRgb->GetData(MEMORYDEVICE_CUDA);
  const Vector4i *offsets_depth = m_offsetsDepth->GetData(MEMORYDEVICE_CUDA);

  Vector2i in_dims = rgb_image->noDims;
  Vector2i out_dims(rgb_image->noDims.x / m_featureStep,
      rgb_image->noDims.y / m_featureStep);

  if (features_image->noDims != out_dims) // Just for the call to Clear()
  {
    features_image->ChangeDims(out_dims);
    features_image->Clear();
  }

  RGBDPatchFeature *features = features_image->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(32, 32);
  dim3 gridSize((out_dims.x + blockSize.x - 1) / blockSize.x,
      (out_dims.y + blockSize.y - 1) / blockSize.y);

  // The colour feature calculator also computed the 3D pose of the point
  // and fills the colour in the feature struct. Intrinsics are only needed here
  ck_compute_colour_feature<<<gridSize, blockSize>>>(features, rgb, depth, offsets_rgb, channels_rgb,
      in_dims, out_dims, intrinsics, cameraPose, m_featureStep, m_normalizeRgb);
  ORcudaKernelCheck;

  ck_compute_depth_feature<<<gridSize, blockSize>>>(features, depth, offsets_depth,
      in_dims, out_dims, m_featureStep, m_normalizeDepth);
  ORcudaKernelCheck;
}

}
