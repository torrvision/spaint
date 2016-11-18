/**
 * spaint: RGBDPatchFeatureCalculator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR

#include <boost/shared_ptr.hpp>
#include "RGBDPatchFeature.h"
#include "ITMLib/Utils/ITMImageTypes.h"

namespace spaint
{

class RGBDPatchFeatureCalculator
{
public:
  RGBDPatchFeatureCalculator();
  virtual ~RGBDPatchFeatureCalculator();

  virtual void compute_feature(const ITMUChar4Image *rgbImage,
      const ITMFloatImage *depthImage, const Vector4f &intrinsics,
      RGBDPatchFeatureImage *featuresImage,
      const Matrix4f &cameraPose) const = 0;

  void compute_feature(const ITMUChar4Image *rgbImage,
      const ITMFloatImage *depthImage, const Vector4f &intrinsics,
      RGBDPatchFeatureImage *featuresImage) const;

  void set_feature_step(uint32_t featureStep);
  uint32_t get_feature_step() const;

protected:
  uint32_t m_featureStep;

  boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > m_offsetsRgb;
  boost::shared_ptr<ORUtils::MemoryBlock<uchar> > m_channelsRgb;
  bool m_normalizeRgb;

  boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > m_offsetsDepth;
  bool m_normalizeDepth;
};

typedef boost::shared_ptr<const RGBDPatchFeatureCalculator> RGBDPatchFeatureCalculator_CPtr;
}

#endif
