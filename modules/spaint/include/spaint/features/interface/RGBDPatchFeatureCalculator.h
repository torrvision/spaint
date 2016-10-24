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

  virtual void ComputeFeature(const ITMUChar4Image_CPtr &rgb_image,
      const ITMFloatImage_CPtr &depth, const Vector4f &intrinsics,
      RGBDPatchFeatureImage_Ptr &features,
      const Matrix4f &camera_pose) const = 0;

  void ComputeFeature(const ITMUChar4Image_CPtr &rgb_image,
      const ITMFloatImage_CPtr &depth, const Vector4f &intrinsics,
      RGBDPatchFeatureImage_Ptr &features) const;

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
