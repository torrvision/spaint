/**
 * spaint: RGBDPatchFeatureCalculator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR

#include "RGBDPatchFeature.h"

namespace spaint
{

class RGBDPatchFeatureCalculator
{
public:
  RGBDPatchFeatureCalculator();
  virtual ~RGBDPatchFeatureCalculator();

  virtual void ComputeFeature(const ITMUChar4Image *rgb_image,
      const ITMFloatImage *depth_image, const Vector4f &intrinsics,
      RGBDPatchFeatureImage *features_image,
      const Matrix4f &cameraPose) const = 0;

  void ComputeFeature(const ITMUChar4Image *rgb_image,
      const ITMFloatImage *depth_image, const Vector4f &intrinsics,
      RGBDPatchFeatureImage *features_image) const;

protected:
  int m_featureStep;

  boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > m_offsetsRgb;
  boost::shared_ptr<ORUtils::MemoryBlock<uchar> > m_channelsRgb;
  bool m_normalizeRgb;

  boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > m_offsetsDepth;
  bool m_normalizeDepth;
};

typedef boost::shared_ptr<const RGBDPatchFeatureCalculator> RGBDPatchFeatureCalculator_CPtr;
}

#endif
