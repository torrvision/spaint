/**
 * spaint: RGBDPatchFeatureCalculator_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.h"
#include "features/shared/RGBDPatchFeatureCalculator_Shared.h"

namespace spaint
{
RGBDPatchFeatureCalculator_CPU::RGBDPatchFeatureCalculator_CPU() :
    RGBDPatchFeatureCalculator()
{
}

void RGBDPatchFeatureCalculator_CPU::ComputeFeature(
    const ITMUChar4Image *rgb_image, const ITMFloatImage *depth_image,
    const Vector4f &intrinsics, RGBDPatchFeatureImage *features_image,
    const Matrix4f &cameraPose) const
{
  const Vector4u *rgb = rgb_image->GetData(MEMORYDEVICE_CPU);
  const float *depth = depth_image->GetData(MEMORYDEVICE_CPU);

  const Vector4i *offsets_rgb = m_offsetsRgb->GetData(MEMORYDEVICE_CPU);
  const uchar *channels_rgb = m_channelsRgb->GetData(MEMORYDEVICE_CPU);
  const Vector4i *offsets_depth = m_offsetsDepth->GetData(MEMORYDEVICE_CPU);

  Vector2i in_dims = rgb_image->noDims;
  Vector2i out_dims(rgb_image->noDims.x / m_featureStep,
      rgb_image->noDims.y / m_featureStep);

  if (features_image->noDims != out_dims) // Just for the call to Clear()
  {
    features_image->ChangeDims(out_dims);
    features_image->Clear();
  }

  RGBDPatchFeature *features = features_image->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int yOut = 0; yOut < out_dims.height; ++yOut)
  {
    for (int xOut = 0; xOut < out_dims.width; ++xOut)
    {
      const Vector2i xyOut(xOut, yOut);
      const Vector2i xyIn(xOut * m_featureStep, yOut * m_featureStep);

      compute_colour_patch_feature(features, rgb, depth, offsets_rgb,
          channels_rgb, in_dims, out_dims, intrinsics, cameraPose,
          m_normalizeRgb, xyIn, xyOut);

      compute_depth_patch_feature(features, depth, offsets_depth, in_dims,
          out_dims, m_normalizeDepth, xyIn, xyOut);
    }
  }
}

}
