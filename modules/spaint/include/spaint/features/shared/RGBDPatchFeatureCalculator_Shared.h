/**
 * spaint: RGBDPatchFeatureCalculator_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_RGBDPATCHFEATURECALCULATOR_SHARED
#define H_SPAINT_RGBDPATCHFEATURECALCULATOR_SHARED

#include <ITMLib/Utils/ITMProjectionUtils.h>
#include <ORUtils/PlatformIndependence.h>

#include <helper_math.h>

namespace spaint
{

/**
 * \brief Fill the keypoint and compute the RGB part of the descriptor.
 *
 * \param keypoints     A pointer to the keypoint image to fill.
 * \param descriptors   A pointer to the descriptors image.
 * \param rgb           A pointer to the colour image.
 * \param depths        A pointer to the depth values.
 * \param offsetsRgb    A pointer to the vector of colour offsets used to compute the descriptor.
 * \param channelsRgb   A pointer to the vector storing the colour channels used to compute the descriptor.
 * \param imgSize       The size of the input RGBD image.
 * \param outSize       The size of the output keypoint/descriptor images.
 * \param intrinsics    The depth camera intrinsics.
 * \param cameraPose    The transform bringing points in camera coordinates to the "descriptor" reference frame.
 *                      Note: set to identity when relocalising the frame and to
 *                      the inverse camera pose when adapting the relocalisation forest.
 * \param normalize     Whether the offsets have to be normalized according to the depth in the keypoint pixel.
 * \param xyIn          The pixel in the input image for which the keypoint/descriptor has to be computed.
 * \param xyOut         The position in the output keypoints/descriptor image where to store the computed values.
 */
_CPU_AND_GPU_CODE_
inline void compute_colour_patch_feature(Keypoint3DColour *keypoints,
    RGBDPatchDescriptor *descriptors, const Vector4u *rgb, const float *depths,
    const Vector4i *offsetsRgb, const uchar *channelsRgb,
    const Vector2i &imgSize, const Vector2i &outSize,
    const Vector4f &intrinsics, const Matrix4f &cameraPose, bool normalize,
    const Vector2i &xyIn, const Vector2i &xyOut)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;
  const float depth = depths[linearIdxIn];

  // References to the output storage.
  Keypoint3DColour &outKeypoint = keypoints[linearIdxOut];
  RGBDPatchDescriptor &outFeature = descriptors[linearIdxOut];

  if (depth <= 0.f)
  {
    // Mark as invalid and return.
    outKeypoint.valid = false;
    return;
  }

  // Compute keypoint position in camera frame.
  const Vector3f position = reproject(xyIn, depth, intrinsics);

  // Bring position to "descriptor frame".
  outKeypoint.position = cameraPose * position;

  // Copy the colour for future reference.
  outKeypoint.colour = rgb[linearIdxIn].toVector3();

  // Mark the keypoint as valid
  outKeypoint.valid = true;

  // Compute the differences and fill the descriptor.
  for (int featIdx = 0; featIdx < RGBDPatchDescriptor::RGB_FEATURE_COUNT;
      ++featIdx)
  {
    const int channel = channelsRgb[featIdx];
    const Vector4i offset = offsetsRgb[featIdx];

    // Secondary points used when computing the differences.
    int x1, y1;
    //    int x2, y2;

    if (normalize)
    {
      // Normalize the offset by the depth of the central pixel
      // and clamp the result to the actual image size.
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
      // Force the secondary point to be inside the image plane.
      x1 = min(max(xyIn.x + offset[0], 0), imgSize.width - 1);
      y1 = min(max(xyIn.y + offset[1], 0), imgSize.height - 1);
      //      x2 = min(max(xy_in.x + offset[2], 0), img_size.width - 1);
      //      y2 = min(max(xy_in.y + offset[3], 0), img_size.height - 1);
    }

    // Linear index of the pixel identified by the offset.
    const int linear_1 = y1 * imgSize.x + x1;
    //    const int linear_2 = y2 * img_size.x + x2;

    // This would be the correct definition but scoreforests's code has the other one
    //    descriptors[linear_idx].rgb[feat_idx] =
    //        rgb[linear_1][channel] - rgb[linear_2][channel];

    outFeature.rgb[featIdx] = rgb[linear_1][channel]
        - rgb[linearIdxIn][channel];
  }
}

/**
 * \brief Compute the depth part of the descriptor.
 *
 * \param keypoints     A pointer to the keypoint image to fill.
 * \param descriptors   A pointer to the descriptors image.
 * \param depths        A pointer to the depth values.
 * \param offsetsDepth  A pointer to the vector of depth offsets used to compute the descriptor.
 * \param imgSize       The size of the input RGBD image.
 * \param outSize       The size of the output keypoint/descriptor images.
 * \param normalize     Whether the offsets have to be normalized according to the depth in the keypoint pixel.
 * \param xyIn          The pixel in the input image for which the keypoint/descriptor has to be computed.
 * \param xyOut         The position in the output keypoints/descriptor image where to store the computed values.
 */
_CPU_AND_GPU_CODE_
inline void compute_depth_patch_feature(Keypoint3DColour *keypoints,
    RGBDPatchDescriptor *features, const float *depths,
    const Vector4i *offsetsDepth, const Vector2i &imgSize,
    const Vector2i &outSize, bool normalize, const Vector2i &xyIn,
    const Vector2i &xyOut)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;
  const float depth = depths[linearIdxIn];

  if (depth <= 0.f)
  {
    // No need to mark the keypoint as invalid, compute_colour_patch_feature already does that.
    return;
  }

  // Compute the differences and fill the descriptor.
  for (int featIdx = 0; featIdx < RGBDPatchDescriptor::DEPTH_FEATURE_COUNT;
      ++featIdx)
  {
    const Vector4i offset = offsetsDepth[featIdx];

    // Secondary points used when computing the differences.
    int x1, y1;
    //    int x2, y2;

    if (normalize)
    {
      // Normalize the offset and clamp the coordinates inside the image bounds.
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
      // Just clamp the secondary point to the image bounds.
      x1 = min(max(xyIn.x + offset[0], 0), imgSize.width - 1);
      y1 = min(max(xyIn.y + offset[1], 0), imgSize.height - 1);
      //      x2 = min(max(xy_in.x + offset[2], 0), img_size.width - 1);
      //      y2 = min(max(xy_in.y + offset[3], 0), img_size.height - 1);
    }

    // Linear index of the pixel identified by the offset.
    const int linear_1 = y1 * imgSize.x + x1;
    //    const int linear_2 = y2 * img_size.x + x2;

    // Depth in mm of the central point.
    const float depth_mm = depth * 1000.f;
    // Max because ITM sometimes has invalid depths stored as -1
    const float depth_1_mm = max(depths[linear_1] * 1000.f, 0.f);

    // Again, this would be the correct definition but scoreforests's code has the other one.
    //    features[linear_idx].depth[feat_idx] =
    //        depths[linear_1] * 1000.f - depths[linear_2] * 1000.f;

    // As for colour, the implementation differs from the paper
    features[linearIdxOut].depth[featIdx] = depth_1_mm - depth_mm;
  }
}
}

#endif
