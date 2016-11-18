/**
 * spaint: RGBDPatchFeatureCalculator_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATORCUDA
#define H_SPAINT_RGBDPATCHFEATURECALCULATORCUDA

#include "../interface/RGBDPatchFeatureCalculator.h"

namespace spaint
{
class RGBDPatchFeatureCalculator_CUDA: public RGBDPatchFeatureCalculator
{
public:
  RGBDPatchFeatureCalculator_CUDA();

  virtual void compute_feature(const ITMUChar4Image *rgbImage,
      const ITMFloatImage *depthImage, const Vector4f &intrinsics,
      RGBDPatchFeatureImage *featuresImage, const Matrix4f &cameraPose) const;
};
}

#endif
