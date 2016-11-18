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

void RGBDPatchFeatureCalculator_CPU::compute_feature(
    const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage,
    const Vector4f &intrinsics, RGBDPatchFeatureImage *featuresImage,
    const Matrix4f &cameraPose) const
{
  const Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  const float *depth = depthImage->GetData(MEMORYDEVICE_CPU);

  const Vector4i *offsetsRgb = m_offsetsRgb->GetData(MEMORYDEVICE_CPU);
  const uchar *channelsRgb = m_channelsRgb->GetData(MEMORYDEVICE_CPU);
  const Vector4i *offsetsDepth = m_offsetsDepth->GetData(MEMORYDEVICE_CPU);

  Vector2i inDims = rgbImage->noDims;
  Vector2i outDims(rgbImage->noDims.x / m_featureStep,
      rgbImage->noDims.y / m_featureStep);

  featuresImage->ChangeDims(outDims);
  RGBDPatchFeature *features = featuresImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int yOut = 0; yOut < outDims.height; ++yOut)
  {
    for (int xOut = 0; xOut < outDims.width; ++xOut)
    {
      const Vector2i xyOut(xOut, yOut);
      const Vector2i xyIn(xOut * m_featureStep, yOut * m_featureStep);

      compute_colour_patch_feature(features, rgb, depth, offsetsRgb,
          channelsRgb, inDims, outDims, intrinsics, cameraPose, m_normalizeRgb,
          xyIn, xyOut);

      compute_depth_patch_feature(features, depth, offsetsDepth, inDims,
          outDims, m_normalizeDepth, xyIn, xyOut);
    }
  }
}

}
