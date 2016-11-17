/**
 * spaint: RGBDPatchFeature.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURE
#define H_SPAINT_RGBDPATCHFEATURE

#include "ITMLib/Utils/ITMMath.h"
#include "ORUtils/Image.h"
#include "ORUtils/PlatformIndependence.h"
#include <boost/shared_ptr.hpp>

namespace spaint
{

struct RGBDPatchFeature
{
  static const int DEPTH_OFFSET = 0;
  static const int DEPTH_FEATURE_COUNT = 128;
  static const int RGB_OFFSET = 128;
  static const int RGB_FEATURE_COUNT = 128;
  static const int FEATURE_SIZE = RGB_FEATURE_COUNT + DEPTH_FEATURE_COUNT;

  union
  {
    float data[FEATURE_SIZE];
    struct
    {
      float depth[DEPTH_FEATURE_COUNT];
      float rgb[RGB_FEATURE_COUNT];
    };
  };

  Vector4f position;
  Vector3u colour;

  _CPU_AND_GPU_CODE_
  inline bool valid() const
  {
    return position.w >= 0.f;
  }
};

typedef ORUtils::Image<RGBDPatchFeature> RGBDPatchFeatureImage;
typedef boost::shared_ptr<ORUtils::Image<RGBDPatchFeature> > RGBDPatchFeatureImage_Ptr;
typedef boost::shared_ptr<const ORUtils::Image<RGBDPatchFeature> > RGBDPatchFeatureImage_CPtr;

}

#endif
