/**
 * spaint: RGBDPatchFeatureCalculator_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR_SHARED
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR_SHARED

namespace spaint
{

_CPU_AND_GPU_CODE_
inline void compute_colour_patch_feature(RGBDPatchFeature *features,
    const Vector4u *rgb, const float *depths, const Vector4i *offsets_rgb,
    const uchar *channels_rgb, Vector2i img_size, bool normalize, int x, int y)
{
  const int linear_idx = y * img_size.x + x;
  const float depth = depths[linear_idx];

  if (depth <= 0.f)
    return;

  for (int feat_idx = 0; feat_idx < RGBDPatchFeature::RGB_FEATURE_COUNT;
      ++feat_idx)
  {
    const int channel = channels_rgb[feat_idx];
    const Vector4i offset = offsets_rgb[feat_idx];

    int x1, y1, x2, y2;

    if (normalize)
    {
      x1 = min(max(x + static_cast<int>(offset[0] / depth), 0),
          img_size.width - 1);
      y1 = min(max(y + static_cast<int>(offset[1] / depth), 0),
          img_size.height - 1);
      x2 = min(max(x + static_cast<int>(offset[2] / depth), 0),
          img_size.width - 1);
      y2 = min(max(y + static_cast<int>(offset[3] / depth), 0),
          img_size.height - 1);
    }
    else
    {
      x1 = min(max(x + offset[0], 0), img_size.width - 1);
      y1 = min(max(y + offset[1], 0), img_size.height - 1);
      x2 = min(max(x + offset[2], 0), img_size.width - 1);
      y2 = min(max(y + offset[3], 0), img_size.height - 1);
    }

    const int linear_1 = y1 * img_size.x + x1;
    const int linear_2 = y2 * img_size.x + x2;

    features[linear_idx].data[RGBDPatchFeature::RGB_OFFSET + feat_idx] =
        rgb[linear_1][channel] - rgb[linear_2][channel];
  }
}
}

#endif
