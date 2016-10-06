/**
 * spaint: RGBDPatchFeatureCalculator_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATORCUDA
#define H_SPAINT_RGBDPATCHFEATURECALCULATORCUDA

#include "../interface/RGBDPatchFeatureCalculator.h"

namespace spaint
{
class RGBDPatchFeatureCalculator_CUDA : public RGBDPatchFeatureCalculator
{
public:
  RGBDPatchFeatureCalculator_CUDA();

  virtual void ComputeFeature(const ITMUChar4Image_CPtr &rgb_image,
      const ITMFloatImage_CPtr &depth,
      boost::shared_ptr<ORUtils::Image<RGBDPatchFeature> > &features) const;
};
}

#endif
