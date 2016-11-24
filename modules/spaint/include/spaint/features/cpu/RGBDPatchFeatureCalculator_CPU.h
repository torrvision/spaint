/**
 * spaint: RGBDPatchFeatureCalculator_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATORCPU
#define H_SPAINT_RGBDPATCHFEATURECALCULATORCPU

#include "../interface/RGBDPatchFeatureCalculator.h"

namespace spaint
{
class RGBDPatchFeatureCalculator_CPU: public RGBDPatchFeatureCalculator
{
public:
  RGBDPatchFeatureCalculator_CPU();

  virtual void compute_feature(const ITMUChar4Image *rgbImage,
      const ITMFloatImage *depthImage, const Vector4f &intrinsics,
      Keypoint3DColourImage *keypointsImage,
      RGBDPatchDescriptorImage *featuresImage,
      const Matrix4f &cameraPose) const;
};
}

#endif
