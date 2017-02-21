/**
 * grove: RGBDPatchFeatureCalculator_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RGBDPATCHFEATURECALCULATOR_SHARED
#define H_GROVE_RGBDPATCHFEATURECALCULATOR_SHARED

#include <helper_math.h>

#include <ITMLib/Utils/ITMProjectionUtils.h>

#include <ORUtils/PlatformIndependence.h>

namespace grove {

/**
 * \brief Compute the keypoint according to the input images.
 *
 * \param keypoints     A pointer to the keypoint image to fill.
 * \param rgb           A pointer to the colour image.
 * \param depths        A pointer to the depth values.
 * \param intrinsics    The depth camera intrinsics.
 * \param imgSize       The size of the input RGBD image.
 * \param outSize       The size of the output keypoint image.
 * \param xyIn          The pixel in the input image for which the keypoint has to be computed.
 * \param xyOut         The position in the output keypoint image where to store the computed values.
 * \param cameraPose    The transform bringing points in camera coordinates to the "descriptor" reference frame.
 *                      Note: set to identity when relocalising the frame and to
 *                      the inverse camera pose when adapting the relocalisation forest.
 */
template<typename KeypointType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_keypoint(KeypointType *keypoints,
    const Vector4u *rgb, const float *depths, const Vector4f &intrinsics,
    const Vector2i &imgSize, const Vector2i &outSize,
    const Vector2i &xyIn, const Vector2i &xyOut,
    const Matrix4f &cameraPose);

/**
 * \brief Compute the keypoint according to the input images.
 *
 * Specialisation for 3D keypoints. The coordinates are in the local/global frame depending on cameraPose.
 * Validity depends on the availability of depth informations.
 */
template<>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_keypoint(Keypoint3DColour *keypoints,
    const Vector4u *rgb, const float *depths, const Vector4f &intrinsics,
    const Vector2i &imgSize, const Vector2i &outSize,
    const Vector2i &xyIn, const Vector2i &xyOut,
    const Matrix4f &cameraPose)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;
  const float depth = depths[linearIdxIn];

  // References to the output storage.
  Keypoint3DColour &outKeypoint = keypoints[linearIdxOut];

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

  // Copy the colour for future reference (reset it if the image is empty).
  outKeypoint.colour = rgb ? rgb[linearIdxIn].toVector3() : Vector3u(0, 0, 0);

  // Mark the keypoint as valid
  outKeypoint.valid = true;
}

/**
 * \brief Compute the keypoint according to the input images.
 *
 * Specialisation for 2D keypoints. Always valid.
 */
template<>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_keypoint(Keypoint2D *keypoints,
    const Vector4u *rgb, const float *depths, const Vector4f &intrinsics,
    const Vector2i &imgSize, const Vector2i &outSize,
    const Vector2i &xyIn, const Vector2i &xyOut,
    const Matrix4f &cameraPose)
{
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;

  // References to the output storage.
  Keypoint2D &outKeypoint = keypoints[linearIdxOut];

  // The 2D keypoint is always valid (and represents the input coordinates).
  outKeypoint.position.x = static_cast<float>(xyIn.x);
  outKeypoint.position.y = static_cast<float>(xyIn.y);
  outKeypoint.valid = true;
}

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
 * \param normalise     Whether the offsets have to be normalized according to the depth in the keypoint pixel.
 * \param xyIn          The pixel in the input image for which the keypoint/descriptor has to be computed.
 * \param xyOut         The position in the output keypoints/descriptor image where to store the computed values.
 */
template<typename KeypointType, typename DescriptorType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_colour_patch_feature(const KeypointType *keypoints,
    DescriptorType *descriptors, const Vector4u *rgb, const float *depths,
    const Vector4i *offsetsRgb, const uchar *channelsRgb,
    const Vector2i &imgSize, const Vector2i &outSize, bool normalise,
    const Vector2i &xyIn, const Vector2i &xyOut, uint32_t featuresCount,
    uint32_t outputFeaturesOffset)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;

  const KeypointType &outKeypoint = keypoints[linearIdxOut];
  if (!outKeypoint.valid)
  {
    return;
  }

  // If normalisation is turned off or the depth image is invalid set the depth for the current pixel to 0.
  // outKeypoint should have been invalid if we actually needed that information.
  const float depth = (depths && normalise) ? depths[linearIdxIn] : 0.f;

  // Compute the differences and fill the descriptor.
  for (uint32_t featIdx = 0; featIdx < featuresCount;
      ++featIdx)
  {
    const int channel = channelsRgb[featIdx];
    const Vector4i offset = offsetsRgb[featIdx];

    // Secondary points used when computing the differences.
    int x1, y1;
    //    int x2, y2;

    if (normalise)
    {
      // Normalise the offset by the depth of the central pixel
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

    // References to the output storage.
    DescriptorType &outFeature = descriptors[linearIdxOut];
    // This would be the correct definition but scoreforests's code has the other one
    //    outFeature.data[outputFeaturesOffset + featIdx] =
    //        rgb[linear_1][channel] - rgb[linear_2][channel];

    outFeature.data[outputFeaturesOffset + featIdx] = rgb[linear_1][channel]
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
 * \param intrinsics    The depth camera intrinsics.
 * \param cameraPose    The transform bringing points in camera coordinates to the "descriptor" reference frame.
 *                      Note: set to identity when relocalising the frame and to
 *                      the inverse camera pose when adapting the relocalisation forest.
 * \param normalise     Whether the offsets have to be normalised according to the depth in the keypoint pixel.
 * \param xyIn          The pixel in the input image for which the keypoint/descriptor has to be computed.
 * \param xyOut         The position in the output keypoints/descriptor image where to store the computed values.
 */
template<typename KeypointType, typename DescriptorType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_depth_patch_feature(const KeypointType *keypoints,
    DescriptorType *features, const float *depths,
    const Vector4i *offsetsDepth, const Vector2i &imgSize,
    const Vector2i &outSize, const Vector4f &intrinsics,
    const Matrix4f &cameraPose, bool normalise, const Vector2i &xyIn,
    const Vector2i &xyOut, uint32_t featuresCount,
    uint32_t outputFeaturesOffset)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;

  // References to the output storage.
  const KeypointType &outKeypoint = keypoints[linearIdxOut];
  if (!outKeypoint.valid)
  {
    return;
  }

  // Must be valid, outKeypoint would have been invalid otherwise.
  const float depth = depths[linearIdxIn];

  // Compute the differences and fill the descriptor.
  for (uint32_t featIdx = 0; featIdx < featuresCount;
      ++featIdx)
  {
    const Vector4i offset = offsetsDepth[featIdx];

    // Secondary points used when computing the differences.
    int x1, y1;
    //    int x2, y2;

    if (normalise)
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

    DescriptorType &outFeature = features[linearIdxOut];
    // Again, this would be the correct definition but scoreforests's code has the other one.
    //    outFeature.data[outputFeaturesOffset + featIdx] =
    //        depths[linear_1] * 1000.f - depths[linear_2] * 1000.f;

    // As for colour, the implementation differs from the paper
    outFeature.data[outputFeaturesOffset + featIdx] = depth_1_mm - depth_mm;
  }
}
}

#endif
