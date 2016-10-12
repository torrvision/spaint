/**
 * spaint: RGBDPatchFeatureCalculator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR

#include "../../util/ITMImagePtrTypes.h"

namespace spaint
{
struct RGBDPatchFeature
{
  static const int RGB_OFFSET = 0;
  static const int RGB_FEATURE_COUNT = 128;
  static const int DEPTH_OFFSET = 128;
  static const int DEPTH_FEATURE_COUNT = 128;
  static const int FEATURE_SIZE = RGB_FEATURE_COUNT + DEPTH_FEATURE_COUNT;

  union
  {
    float data[FEATURE_SIZE];
    struct
    {
      float rgb[RGB_FEATURE_COUNT];
      float depth[DEPTH_FEATURE_COUNT];
    };
  };
};

class RGBDPatchFeatureCalculator
{
public:
  RGBDPatchFeatureCalculator();
  virtual ~RGBDPatchFeatureCalculator();

  virtual void ComputeFeature(const ITMUChar4Image_CPtr &rgb_image,
      const ITMFloatImage_CPtr &depth,
      boost::shared_ptr<ORUtils::Image<RGBDPatchFeature> > &features) const = 0;

protected:
  boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > m_offsetsRgb;
  boost::shared_ptr<ORUtils::MemoryBlock<uchar> > m_channelsRgb;
  bool m_normalizeRgb;

  boost::shared_ptr<ORUtils::MemoryBlock<Vector4i> > m_offsetsDepth;
  bool m_normalizeDepth;
};

typedef boost::shared_ptr<const RGBDPatchFeatureCalculator> RGBDPatchFeatureCalculator_CPtr;
}

#endif
