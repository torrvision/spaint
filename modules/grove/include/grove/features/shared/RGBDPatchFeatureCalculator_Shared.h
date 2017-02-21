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
  const int linearIdxIn = xyIn.y * inSize.width + xyIn.x;

  // Look up the keypoint corresponding to the specified pixel, and early out if it's not valid.
  const int linearIdxOut = xyOut.y * outSize.width + xyOut.x;
  const KeypointType& keypoint = keypoints[linearIdxOut];
  if(!keypoint.valid) return;

  // If we're normalising the RGB offsets based on depth and depth information is available,
  // look up the depth for the input pixel; otherwise, default to 1.
  const float depth = (normalise && depths) ? depths[linearIdxIn] : 1.0f;

  // Compute the differences and fill the descriptor.
  for(uint32_t featIdx = 0; featIdx < rgbFeatureCount; ++featIdx)
  {
    const int channel = rgbChannels[featIdx];
    const Vector4i offset = rgbOffsets[featIdx];

    // Secondary points used when computing the differences.
    int x1, y1;
    //    int x2, y2;

    if (normalise)
    {
      // Normalise the offset by the depth of the central pixel
      // and clamp the result to the actual image size.
      x1 = min(max(xyIn.x + static_cast<int>(offset[0] / depth), 0),
          inSize.width - 1);
      y1 = min(max(xyIn.y + static_cast<int>(offset[1] / depth), 0),
          inSize.height - 1);
      //      x2 = min(max(xy_in.x + static_cast<int>(offset[2] / depth), 0),
      //          img_size.width - 1);
      //      y2 = min(max(xy_in.y + static_cast<int>(offset[3] / depth), 0),
      //          img_size.height - 1);
    }
    else
    {
      // Force the secondary point to be inside the image plane.
      x1 = min(max(xyIn.x + offset[0], 0), inSize.width - 1);
      y1 = min(max(xyIn.y + offset[1], 0), inSize.height - 1);
      //      x2 = min(max(xy_in.x + offset[2], 0), img_size.width - 1);
      //      y2 = min(max(xy_in.y + offset[3], 0), img_size.height - 1);
    }

    // Linear index of the pixel identified by the offset.
    const int linear_1 = y1 * inSize.x + x1;
    //    const int linear_2 = y2 * img_size.x + x2;

    // References to the output storage.
    DescriptorType& outFeature = descriptors[linearIdxOut];
    // This would be the correct definition but scoreforests's code has the other one
    //    outFeature.data[outputFeaturesOffset + featIdx] =
    //        rgb[linear_1][channel] - rgb[linear_2][channel];

    outFeature.data[rgbFeatureOffset + featIdx] = rgb[linear_1][channel] - rgb[linearIdxIn][channel];
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
template <typename KeypointType, typename DescriptorType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_depth_features(const KeypointType *keypoints, DescriptorType *features, const float *depths, const Vector4i *offsetsDepth,
                                   const Vector2i& imgSize, const Vector2i& outSize, const Vector4f& intrinsics, const Matrix4f& cameraPose,
                                   bool normalise, const Vector2i& xyIn, const Vector2i& xyOut, uint32_t featuresCount, uint32_t outputFeaturesOffset)
{
  const int linearIdxIn = xyIn.y * imgSize.x + xyIn.x;
  const int linearIdxOut = xyOut.y * outSize.x + xyOut.x;

  // References to the output storage.
  const KeypointType &outKeypoint = keypoints[linearIdxOut];
  if(!outKeypoint.valid)
  {
    return;
  }

  // Must be valid, outKeypoint would have been invalid otherwise.
  const float depth = depths[linearIdxIn];

  // Compute the differences and fill the descriptor.
  for(uint32_t featIdx = 0; featIdx < featuresCount; ++featIdx)
  {
    const Vector4i offset = offsetsDepth[featIdx];

    // Secondary points used when computing the differences.
    int x1, y1;
    //    int x2, y2;

    if(normalise)
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
