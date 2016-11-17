/**
 * spaint: RGBDPatchFeatureCalculator_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR_SHARED
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR_SHARED

#include "helper_math.h"
#include "ORUtils/PlatformIndependence.h"

namespace spaint
{

_CPU_AND_GPU_CODE_
inline void compute_colour_patch_feature(RGBDPatchFeature *features,
    const Vector4u *rgb, const float *depths, const Vector4i *offsetsRgb,
    const uchar *channelsRgb, const Vector2i &imgSize, const Vector2i &outSize,
    const Vector4f &intrinsics, const Matrix4f &cameraPose, bool normalize,
    const Vector2i &xyIn, const Vector2i &xyOut)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const float depth = depths[linearIdxIn];

  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;
  RGBDPatchFeature &outFeature = features[linearIdxOut];

  if (depth <= 0.f)
  {
    // Mark as invalid
    outFeature.position.w = -1.f;
    outFeature.colour.w = -1.f;
    return;
  }

  // Compute position in camera frame
  const Vector4f position(
      depth * ((static_cast<float>(xyIn.x) - intrinsics.z) / intrinsics.x),
      depth * ((static_cast<float>(xyIn.y) - intrinsics.w) / intrinsics.y),
      depth, 1.0f);

  // Bring position in "world frame"
  outFeature.position = cameraPose * position;

  // Copy the colour for future reference
  outFeature.colour = rgb[linearIdxIn].toFloat();

  for (int featIdx = 0; featIdx < RGBDPatchFeature::RGB_FEATURE_COUNT;
      ++featIdx)
  {
    const int channel = channelsRgb[featIdx];
    const Vector4i offset = offsetsRgb[featIdx];

    int x1, y1;
//    int x2, y2;

    if (normalize)
    {
      x1 = min(max(xyIn.x + static_cast<int>(offset[0] / depth), 0),
          imgSize.width - 1);
      y1 = min(max(xyIn.y + static_cast<int>(offset[1] / depth), 0),
          imgSize.height - 1);
//      x2 = min(max(xy_in.x + static_cast<int>(offset[2] / depth), 0),
//          img_size.width - 1);
//      y2 = min(max(xy_in.y + static_cast<int>(offset[3] / depth), 0),
//          img_size.height - 1);
    }
    else
    {
      x1 = min(max(xyIn.x + offset[0], 0), imgSize.width - 1);
      y1 = min(max(xyIn.y + offset[1], 0), imgSize.height - 1);
//      x2 = min(max(xy_in.x + offset[2], 0), img_size.width - 1);
//      y2 = min(max(xy_in.y + offset[3], 0), img_size.height - 1);
    }

    const int linear_1 = y1 * imgSize.x + x1;
//    const int linear_2 = y2 * img_size.x + x2;

// This would be the correct definition but scoreforests's code has the other one
//    features[linear_idx].rgb[feat_idx] =
//        rgb[linear_1][channel] - rgb[linear_2][channel];

    outFeature.rgb[featIdx] = rgb[linear_1][channel]
        - rgb[linearIdxIn][channel];
  }
}

_CPU_AND_GPU_CODE_
inline void compute_depth_patch_feature(RGBDPatchFeature *features,
    const float *depths, const Vector4i *offsetsDepth, const Vector2i &imgSize,
    const Vector2i &outSize, bool normalize, const Vector2i &xyIn,
    const Vector2i &xyOut)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const float depth = depths[linearIdxIn];

  if (depth <= 0.f)
    return;

  for (int featIdx = 0; featIdx < RGBDPatchFeature::DEPTH_FEATURE_COUNT;
      ++featIdx)
  {
    const Vector4i offset = offsetsDepth[featIdx];

    int x1, y1;
//    int x2, y2;

    if (normalize)
    {
      x1 = min(max(xyIn.x + static_cast<int>(offset[0] / depth), 0),
          imgSize.width - 1);
      y1 = min(max(xyIn.y + static_cast<int>(offset[1] / depth), 0),
          imgSize.height - 1);
//      x2 = min(max(xy_in.x + static_cast<int>(offset[2] / depth), 0),
//          img_size.width - 1);
//      y2 = min(max(xy_in.y + static_cast<int>(offset[3] / depth), 0),
//          img_size.height - 1);
    }
    else
    {
      x1 = min(max(xyIn.x + offset[0], 0), imgSize.width - 1);
      y1 = min(max(xyIn.y + offset[1], 0), imgSize.height - 1);
//      x2 = min(max(xy_in.x + offset[2], 0), img_size.width - 1);
//      y2 = min(max(xy_in.y + offset[3], 0), img_size.height - 1);
    }

    const int linear_1 = y1 * imgSize.x + x1;
//    const int linear_2 = y2 * img_size.x + x2;

    const float depth_mm = depth * 1000.f;
    // because ITM sometimes has invalid depths stored as -1
    const float depth_1_mm = max(depths[linear_1] * 1000.f, 0.f);

    // Features are computed in mm, so we multiply the depth by 1000.
//    features[linear_idx].depth[feat_idx] =
//        depths[linear_1] * 1000.f - depths[linear_2] * 1000.f;

// As for colour, the implementation differs from the paper
    const int linear_idx_out = xyOut.y * outSize.x + xyOut.x;
    features[linear_idx_out].depth[featIdx] = depth_1_mm - depth_mm;
  }
}
}

#endif
