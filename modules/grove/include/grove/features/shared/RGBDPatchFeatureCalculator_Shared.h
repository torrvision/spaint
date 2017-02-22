/**
 * grove: RGBDPatchFeatureCalculator_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RGBDPATCHFEATURECALCULATOR_SHARED
#define H_GROVE_RGBDPATCHFEATURECALCULATOR_SHARED

#include <helper_math.h>

#include <ITMLib/Utils/ITMProjectionUtils.h>

#include <ORUtils/PlatformIndependence.h>

// Whether or not to use the "correct" features, rather than the ones from the SCoRe Forests code.
#define USE_CORRECT_FEATURES 0

namespace grove {

/**
 * \brief Computes colour features for a pixel in the RGBD image and writes them into the relevant descriptor.
 *
 * \param xyIn              The pixel in the RGBD image for which to compute colour features.
 * \param xyOut             The position in the descriptors image into which to write the computed features.
 * \param inSize            The size of the RGBD image.
 * \param outSize           The size of the keypoints/descriptors images.
 * \param rgb               A pointer to the colour image.
 * \param depths            A pointer to the depth image.
 * \param rgbOffsets        A pointer to the vector of offsets needed to specify the colour features to be computed.
 * \param rgbChannels       A pointer to the vector of colour channels needed to specify the colour features to be computed.
 * \param keypoints         A pointer to the keypoints image.
 * \param rgbFeatureCount   The number of colour features to be computed.
 * \param rgbFeatureOffset  The starting offset of the colour features in the feature descriptor.
 * \param normalise         Whether or not to normalise the RGB offsets by the RGBD pixel's depth value.
 * \param descriptors       A pointer to the descriptors image.
 */
template <typename KeypointType, typename DescriptorType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_colour_features(const Vector2i& xyIn, const Vector2i& xyOut, const Vector2i& inSize, const Vector2i& outSize,
                                    const Vector4u *rgb, const float *depths, const Vector4i *rgbOffsets, const uchar *rgbChannels,
                                    const KeypointType *keypoints, const uint32_t rgbFeatureCount, const uint32_t rgbFeatureOffset,
                                    const bool normalise, DescriptorType *descriptors)
{
  // Look up the keypoint corresponding to the specified pixel, and early out if it's not valid.
  const int linearIdxOut = xyOut.y * outSize.width + xyOut.x;
  const KeypointType& keypoint = keypoints[linearIdxOut];
  if(!keypoint.valid) return;

  // If we're normalising the RGB offsets based on depth, and depth information is available,
  // look up the depth for the input pixel; otherwise, default to 1.
  const int linearIdxIn = xyIn.y * inSize.width + xyIn.x;
  const float depth = (normalise && depths) ? depths[linearIdxIn] : 1.0f;

  // Compute the features and fill in the descriptor.
  DescriptorType& descriptor = descriptors[linearIdxOut];
  for(uint32_t featIdx = 0; featIdx < rgbFeatureCount; ++featIdx)
  {
    const int channel = rgbChannels[featIdx];
    const Vector4i offset = rgbOffsets[featIdx];

    // Calculate the position(s) of the secondary point(s) to use when computing the feature.
    int x1, y1;
#if USE_CORRECT_FEATURES
    int x2, y2;
#endif

    // If depth normalisation is turned on, normalise the offset(s) by the depth of the central pixel.
    // Otherwise, just use the offsets as they stand.
    if(normalise)
    {
      x1 = xyIn.x + static_cast<int>(offset[0] / depth);
      y1 = xyIn.y + static_cast<int>(offset[1] / depth);
#if USE_CORRECT_FEATURES
      x2 = xyIn.x + static_cast<int>(offset[2] / depth);
      y2 = xyIn.y + static_cast<int>(offset[3] / depth);
#endif
    }
    else
    {
      x1 = xyIn.x + offset[0];
      y1 = xyIn.y + offset[1];
#if USE_CORRECT_FEATURES
      x2 = xyIn.x + offset[2];
      y2 = xyIn.y + offset[3];
#endif
    }

    // Constrain the secondary point(s) to be within the image.
    x1 = min(max(x1, 0), inSize.width - 1);
    y1 = min(max(y1, 0), inSize.height - 1);
#if USE_CORRECT_FEATURES
    x2 = min(max(x2, 0), inSize.width - 1);
    y2 = min(max(y2, 0), inSize.height - 1);
#endif

    // Calculate the raster position(s) of the secondary point(s).
    const int linear1 = y1 * inSize.width + x1;
#if USE_CORRECT_FEATURES
    const int linear2 = y2 * inSize.width + x2;
#endif

    // Compute the feature and write it into the descriptor.
#if USE_CORRECT_FEATURES
    // This is the "correct" definition, but the SCoRe Forests code uses the other one.
    descriptor.data[rgbFeatureOffset + featIdx] = rgb[linear1][channel] - rgb[linear2][channel];
#else
    // This is the definition used in the SCoRe Forests code.
    descriptor.data[rgbFeatureOffset + featIdx] = rgb[linear1][channel] - rgb[linearIdxIn][channel];
#endif
  }
}

/**
 * \brief Computes depth features for a pixel in the RGBD image and writes them into the relevant descriptor.
 *
 * \param xyIn                The pixel in the RGBD image for which to compute depth features.
 * \param xyOut               The position in the descriptors image into which to write the computed features.
 * \param inSize              The size of the RGBD image.
 * \param outSize             The size of the keypoints/descriptors images.
 * \param depths              A pointer to the depth image.
 * \param depthOffsets        A pointer to the vector of offsets needed to specify the depth features to be computed.
 * \param keypoints           A pointer to the keypoints image.
 * \param depthFeatureCount   The number of depth features to be computed.
 * \param depthFeatureOffset  The starting offset of the depth features in the feature descriptor.
 * \param normalise           Whether or not to normalise the depth offsets by the RGBD pixel's depth value.
 * \param descriptors         A pointer to the descriptors image.
 */
template <typename KeypointType, typename DescriptorType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_depth_features(const Vector2i& xyIn, const Vector2i& xyOut, const Vector2i& inSize, const Vector2i& outSize,
                                   const float *depths, const Vector4i *depthOffsets, const KeypointType *keypoints,
                                   uint32_t depthFeatureCount, uint32_t depthFeatureOffset, bool normalise, DescriptorType *descriptors)
{
  // Look up the keypoint corresponding to the specified pixel, and early out if it's not valid.
  const int linearIdxOut = xyOut.y * outSize.width + xyOut.x;
  const KeypointType& keypoint = keypoints[linearIdxOut];
  if(!keypoint.valid) return;

  // Look up the depth for the input pixel. This must be available, since otherwise the pixel's keypoint would have been invalid.
  const int linearIdxIn = xyIn.y * inSize.width + xyIn.x;
  const float depth = depths[linearIdxIn];

  // Compute the features and fill in the descriptor.
  DescriptorType& descriptor = descriptors[linearIdxOut];
  for(uint32_t featIdx = 0; featIdx < depthFeatureCount; ++featIdx)
  {
    const Vector4i offset = depthOffsets[featIdx];

    // Calculate the position(s) of the secondary point(s) to use when computing the feature.
    int x1, y1;
#if USE_CORRECT_FEATURES
    int x2, y2;
#endif

    // If depth normalisation is turned on, normalise the offset(s) by the depth of the central pixel.
    // Otherwise, just use the offsets as they stand.
    if(normalise)
    {
      x1 = xyIn.x + static_cast<int>(offset[0] / depth);
      y1 = xyIn.y + static_cast<int>(offset[1] / depth);
#if USE_CORRECT_FEATURES
      x2 = xyIn.x + static_cast<int>(offset[2] / depth);
      y2 = xyIn.y + static_cast<int>(offset[3] / depth);
#endif
    }
    else
    {
      x1 = xyIn.x + offset[0];
      y1 = xyIn.y + offset[1];
#if USE_CORRECT_FEATURES
      x2 = xyIn.x + offset[2];
      y2 = xyIn.y + offset[3];
#endif
    }

    // Constrain the secondary point(s) to be within the image.
    x1 = min(max(x1, 0), inSize.width - 1);
    y1 = min(max(y1, 0), inSize.height - 1);
#if USE_CORRECT_FEATURES
    x2 = min(max(x2, 0), inSize.width - 1);
    y2 = min(max(y2, 0), inSize.height - 1);
#endif

    // Calculate the raster position(s) of the secondary point(s).
    const int linear_1 = y1 * inSize.width + x1;
#if USE_CORRECT_FEATURES
    const int linear_2 = y2 * inSize.width + x2;
#endif

    // Depth in mm of the central point.
    const float depth_mm = depth * 1000.f;
    // Max because ITM sometimes has invalid depths stored as -1
    const float depth_1_mm = max(depths[linear_1] * 1000.f, 0.f);

    // Again, this would be the correct definition but scoreforests's code has the other one.
    //    outFeature.data[outputFeaturesOffset + featIdx] =
    //        depths[linear_1] * 1000.f - depths[linear_2] * 1000.f;

    // As for colour, the implementation differs from the paper
    descriptor.data[depthFeatureOffset + featIdx] = depth_1_mm - depth_mm;
  }
}

/**
 * \brief Computes a keypoint for the specified pixel in the RGBD image.
 *
 * \param xyIn        The coordinates of the pixel in the RGBD image for which to compute the keypoint.
 * \param xyOut       The coordinates of the pixel in the keypoints image into which to store the computed keypoint.
 * \param inSize      The size of the RGBD image.
 * \param outSize     The size of the keypoints image.
 * \param rgb         A pointer to the colour image.
 * \param depths      A pointer to the depth image.
 * \param cameraPose  The transform bringing points in camera coordinates to the "descriptor" reference frame.
 *                    This is set to the identity matrix when relocalising the frame and to the inverse camera
 *                    pose when adapting the relocalisation forest.
 * \param intrinsics  The intrinsic parameters for the depth camera.
 * \param keypoints   A pointer to the keypoints image, into which the computed keypoint will be written.
 */
template <typename KeypointType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_keypoint(const Vector2i& xyIn, const Vector2i& xyOut, const Vector2i& inSize, const Vector2i& outSize,
                             const Vector4u *rgb, const float *depths, const Matrix4f& cameraPose, const Vector4f& intrinsics,
                             KeypointType *keypoints);

/**
 * \brief Computes a 2D keypoint for the specified pixel in the RGBD image.
 *
 * The computed keypoint will always be valid.
 */
template <>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_keypoint(const Vector2i& xyIn, const Vector2i& xyOut, const Vector2i& inSize, const Vector2i& outSize,
                             const Vector4u *rgb, const float *depths, const Matrix4f& cameraPose, const Vector4f& intrinsics,
                             Keypoint2D *keypoints)
{
  // Look up the keypoint corresponding to the specified pixel.
  const int linearIdxOut = xyOut.y * outSize.width + xyOut.x;
  Keypoint2D& outKeypoint = keypoints[linearIdxOut];

  // Set its parameters appropriately. Note that 2D keypoints are always valid (and represent the input coordinates).
  outKeypoint.position = xyIn.toFloat();
  outKeypoint.valid = true;
}

/**
 * \brief Computes a 3D keypoint for the specified pixel in the RGBD image.
 *
 * The coordinates are in the local/global frame depending on cameraPose.
 * Validity depends on the availability of depth information.
 */
template <>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_keypoint(const Vector2i& xyIn, const Vector2i& xyOut, const Vector2i& inSize, const Vector2i& outSize,
                             const Vector4u *rgb, const float *depths, const Matrix4f& cameraPose, const Vector4f& intrinsics,
                             Keypoint3DColour *keypoints)
{
  // Look up the keypoint corresponding to the specified pixel.
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;
  Keypoint3DColour& outKeypoint = keypoints[linearIdxOut];

  // Check whether depth is available for the specified pixel. If not, mark the keypoint as invalid and early out.
  const int linearIdxIn = xyIn.y * inSize.x + xyIn.x;
  const float depth = depths[linearIdxIn];
  if(depth <= 0.0f)
  {
    outKeypoint.valid = false;
    return;
  }

  // Back-project the keypoint to determine its position in camera coordinates.
  const Vector3f position = reproject(xyIn, depth, intrinsics);

  // Determine the keypoint's position in "descriptor" coordinates.
  outKeypoint.position = cameraPose * position;

  // Record the pixel's colour in the keypoint for future reference. Default to black if no colour image is available.
  outKeypoint.colour = rgb ? rgb[linearIdxIn].toVector3() : Vector3u(0, 0, 0);

  // Mark the keypoint as valid.
  outKeypoint.valid = true;
}

}

#endif
