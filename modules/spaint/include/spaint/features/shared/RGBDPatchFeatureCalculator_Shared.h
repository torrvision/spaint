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
    const Vector4u *rgb, const float *depths, const Vector4i *offsets_rgb,
    const uchar *channels_rgb, const Vector2i &img_size,
    const Vector2i &out_size, const Vector4f &intrinsics,
    const Matrix4f &cameraPose, bool normalize, const Vector2i &xy_in,
    const Vector2i &xy_out)
{
  const int linear_idx_in = xy_in.y * img_size.x + xy_in.x;
  const float depth = depths[linear_idx_in];

  const int linear_idx_out = xy_out.y * out_size.x + xy_out.x;
  RGBDPatchFeature &out_feature = features[linear_idx_out];

  if (depth <= 0.f)
  {
    // Mark as invalid
    out_feature.position.w = -1.f;
    out_feature.colour.w = -1.f;
    return;
  }

  // Compute position in camera frame
  const Vector4f position(
      depth * ((static_cast<float>(xy_in.x) - intrinsics.z) / intrinsics.x),
      depth * ((static_cast<float>(xy_in.y) - intrinsics.w) / intrinsics.y),
      depth, 1.0f);

  // Bring position in "world frame"
  out_feature.position = cameraPose * position;

  // Copy the colour for future reference
  out_feature.colour = rgb[linear_idx_in].toFloat();

  for (int feat_idx = 0; feat_idx < RGBDPatchFeature::RGB_FEATURE_COUNT;
      ++feat_idx)
  {
    const int channel = channels_rgb[feat_idx];
    const Vector4i offset = offsets_rgb[feat_idx];

    int x1, y1;
//    int x2, y2;

    if (normalize)
    {
      x1 = min(max(xy_in.x + static_cast<int>(offset[0] / depth), 0),
          img_size.width - 1);
      y1 = min(max(xy_in.y + static_cast<int>(offset[1] / depth), 0),
          img_size.height - 1);
//      x2 = min(max(xy_in.x + static_cast<int>(offset[2] / depth), 0),
//          img_size.width - 1);
//      y2 = min(max(xy_in.y + static_cast<int>(offset[3] / depth), 0),
//          img_size.height - 1);
    }
    else
    {
      x1 = min(max(xy_in.x + offset[0], 0), img_size.width - 1);
      y1 = min(max(xy_in.y + offset[1], 0), img_size.height - 1);
//      x2 = min(max(xy_in.x + offset[2], 0), img_size.width - 1);
//      y2 = min(max(xy_in.y + offset[3], 0), img_size.height - 1);
    }

    const int linear_1 = y1 * img_size.x + x1;
//    const int linear_2 = y2 * img_size.x + x2;

    // This would be the correct definition but scoreforests's code has the other one
//    features[linear_idx].rgb[feat_idx] =
//        rgb[linear_1][channel] - rgb[linear_2][channel];

    out_feature.rgb[feat_idx] = rgb[linear_1][channel]
        - rgb[linear_idx_in][channel];
  }
}

_CPU_AND_GPU_CODE_
inline void compute_depth_patch_feature(RGBDPatchFeature *features,
    const float *depths, const Vector4i *offsets_depth,
    const Vector2i &img_size, const Vector2i &out_size, bool normalize,
    const Vector2i &xy_in, const Vector2i &xy_out)
{
  const int linear_idx_in = xy_in.y * img_size.x + xy_in.x;
  const float depth = depths[linear_idx_in];

  if (depth <= 0.f)
    return;

  for (int feat_idx = 0; feat_idx < RGBDPatchFeature::DEPTH_FEATURE_COUNT;
      ++feat_idx)
  {
    const Vector4i offset = offsets_depth[feat_idx];

    int x1, y1;
//    int x2, y2;

    if (normalize)
    {
      x1 = min(max(xy_in.x + static_cast<int>(offset[0] / depth), 0),
          img_size.width - 1);
      y1 = min(max(xy_in.y + static_cast<int>(offset[1] / depth), 0),
          img_size.height - 1);
//      x2 = min(max(xy_in.x + static_cast<int>(offset[2] / depth), 0),
//          img_size.width - 1);
//      y2 = min(max(xy_in.y + static_cast<int>(offset[3] / depth), 0),
//          img_size.height - 1);
    }
    else
    {
      x1 = min(max(xy_in.x + offset[0], 0), img_size.width - 1);
      y1 = min(max(xy_in.y + offset[1], 0), img_size.height - 1);
//      x2 = min(max(xy_in.x + offset[2], 0), img_size.width - 1);
//      y2 = min(max(xy_in.y + offset[3], 0), img_size.height - 1);
    }

    const int linear_1 = y1 * img_size.x + x1;
//    const int linear_2 = y2 * img_size.x + x2;

    const float depth_mm = depth * 1000.f;
    // because ITM sometimes has invalid depths stored as -1
    const float depth_1_mm = max(depths[linear_1] * 1000.f, 0.f);

    // Features are computed in mm, so we multiply the depth by 1000.
//    features[linear_idx].depth[feat_idx] =
//        depths[linear_1] * 1000.f - depths[linear_2] * 1000.f;

// As for colour, the implementation differs from the paper
    const int linear_idx_out = xy_out.y * out_size.x + xy_out.x;
    features[linear_idx_out].depth[feat_idx] = depth_1_mm - depth_mm;
  }
}
}

#endif
