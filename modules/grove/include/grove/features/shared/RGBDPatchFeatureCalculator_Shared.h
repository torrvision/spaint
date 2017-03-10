/**
 * grove: RGBDPatchFeatureCalculator_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RGBDPATCHFEATURECALCULATOR_SHARED
#define H_GROVE_RGBDPATCHFEATURECALCULATOR_SHARED

#include "../interface/RGBDPatchFeatureCalculator.h"

#include <helper_math.h>

#include <ITMLib/Utils/ITMProjectionUtils.h>

#include <ORUtils/PlatformIndependence.h>

namespace grove {

/**
 * \brief Calculates the raster position(s) of the secondary point(s) to use when computing a feature.
 *
 * \param xyIn      The pixel in the RGBD image for which features are being computed.
 * \param offsets   The unnormalised offsets of the secondary point(s) to use when computing the feature.
 * \param inSize    The size of the RGBD image.
 * \param normalise Whether or not to normalise the offsets by the RGBD pixel's depth value.
 * \param depth     The RGBD pixel's depth value.
 * \param raster1   An int into which to write the raster position of the first secondary point.
 * \param raster2   An int into which to write the raster position of the second secondary point (if needed).
 */
template <RGBDPatchFeatureCalculatorDifferenceType DifferenceType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void calculate_secondary_points(const Vector2i& xyIn, const Vector4i& offset, const Vector2i& inSize, const bool normalise, const float depth, int& raster1, int& raster2)
{
  int x1, y1;
  int x2 = 0, y2 = 0;

  // If depth normalisation is turned on, normalise the offset(s) by the depth of the central pixel.
  // Otherwise, just use the offsets as they stand.
  if(normalise)
  {
    x1 = xyIn.x + static_cast<int>(offset[0] / depth);
    y1 = xyIn.y + static_cast<int>(offset[1] / depth);

    if(DifferenceType == RGBDPatchFeatureCalculatorDifferenceType::PAIRWISE_DIFFERENCE)
    {
      x2 = xyIn.x + static_cast<int>(offset[2] / depth);
      y2 = xyIn.y + static_cast<int>(offset[3] / depth);
    }
  }
  else
  {
    x1 = xyIn.x + offset[0];
    y1 = xyIn.y + offset[1];

    if(DifferenceType == RGBDPatchFeatureCalculatorDifferenceType::PAIRWISE_DIFFERENCE)
    {
      x2 = xyIn.x + offset[2];
      y2 = xyIn.y + offset[3];
    }
  }

  // Constrain the secondary point(s) to be within the image.
  x1 = min(max(x1, 0), inSize.width - 1);
  y1 = min(max(y1, 0), inSize.height - 1);

  if(DifferenceType == RGBDPatchFeatureCalculatorDifferenceType::PAIRWISE_DIFFERENCE)
  {
    x2 = min(max(x2, 0), inSize.width - 1);
    y2 = min(max(y2, 0), inSize.height - 1);
  }

  // Calculate the raster position(s) of the secondary point(s).
  raster1 = y1 * inSize.width + x1;
  raster2 = y2 * inSize.width + x2;
}

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
template <RGBDPatchFeatureCalculatorDifferenceType DifferenceType, typename KeypointType, typename DescriptorType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_colour_features(const Vector2i& xyIn, const Vector2i& xyOut, const Vector2i& inSize, const Vector2i& outSize,
                                    const Vector4u *rgb, const float *depths, const Vector4i *rgbOffsets, const uchar *rgbChannels,
                                    const KeypointType *keypoints, const uint32_t rgbFeatureCount, const uint32_t rgbFeatureOffset,
                                    const bool normalise, DescriptorType *descriptors)
{
  // Look up the keypoint corresponding to the specified pixel, and early out if it's not valid.
  const int rasterIdxOut = xyOut.y * outSize.width + xyOut.x;
  const KeypointType& keypoint = keypoints[rasterIdxOut];
  if(!keypoint.valid) return;

  // If we're normalising the RGB offsets based on depth, and depth information is available,
  // look up the depth for the input pixel; otherwise, default to 1.
  const int rasterIdxIn = xyIn.y * inSize.width + xyIn.x;
  const float depth = (normalise && depths) ? depths[rasterIdxIn] : 1.0f;

  // Compute the features and fill in the descriptor.
  DescriptorType& descriptor = descriptors[rasterIdxOut];
  for(uint32_t featIdx = 0; featIdx < rgbFeatureCount; ++featIdx)
  {
    const int channel = rgbChannels[featIdx];
    const Vector4i offsets = rgbOffsets[featIdx];

    // Calculate the raster position(s) of the secondary point(s) to use when computing the feature.
    int raster1, raster2;
    calculate_secondary_points<DifferenceType>(xyIn, offsets, inSize, normalise, depth, raster1, raster2);

    // Compute the feature and write it into the descriptor.
    if(DifferenceType == RGBDPatchFeatureCalculatorDifferenceType::PAIRWISE_DIFFERENCE)
    {
      // This is the "correct" definition, but the SCoRe Forests code uses the other one.
      descriptor.data[rgbFeatureOffset + featIdx] = rgb[raster1][channel] - rgb[raster2][channel];
    }
    else
    {
      // This is the definition used in the SCoRe Forests code.
      descriptor.data[rgbFeatureOffset + featIdx] = static_cast<float>(rgb[raster1][channel] - rgb[rasterIdxIn][channel]);
    }
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
template <RGBDPatchFeatureCalculatorDifferenceType DifferenceType, typename KeypointType, typename DescriptorType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_depth_features(const Vector2i& xyIn, const Vector2i& xyOut, const Vector2i& inSize, const Vector2i& outSize,
                                   const float *depths, const Vector4i *depthOffsets, const KeypointType *keypoints,
                                   uint32_t depthFeatureCount, uint32_t depthFeatureOffset, bool normalise, DescriptorType *descriptors)
{
  // Look up the keypoint corresponding to the specified pixel, and early out if it's not valid.
  const int rasterIdxOut = xyOut.y * outSize.width + xyOut.x;
  const KeypointType& keypoint = keypoints[rasterIdxOut];
  if(!keypoint.valid) return;

  // Look up the depth for the input pixel. This must be available, since otherwise the pixel's keypoint would have been invalid.
  const int rasterIdxIn = xyIn.y * inSize.width + xyIn.x;
  const float depth = depths[rasterIdxIn];

  // Compute the features and fill in the descriptor.
  DescriptorType& descriptor = descriptors[rasterIdxOut];
  for(uint32_t featIdx = 0; featIdx < depthFeatureCount; ++featIdx)
  {
    const Vector4i offsets = depthOffsets[featIdx];

    // Calculate the raster position(s) of the secondary point(s) to use when computing the feature.
    int raster1, raster2;
    calculate_secondary_points<DifferenceType>(xyIn, offsets, inSize, normalise, depth, raster1, raster2);

    // Convert the depth of the first secondary point to millimetres.
    const float depth1Mm = fmaxf(depths[raster1] * 1000.f, 0.0f);  // we use max because InfiniTAM sometimes has invalid depths stored as -1

    // Compute the feature and write it into the descriptor.
    if(DifferenceType == RGBDPatchFeatureCalculatorDifferenceType::PAIRWISE_DIFFERENCE)
    {
      // This is the "correct" definition, but the SCoRe Forests code uses the other one.
      const float depth2Mm = fmaxf(depths[raster2] * 1000.0f, 0.0f);
      descriptor.data[depthFeatureOffset + featIdx] = depth1Mm - depth2Mm;
    }
    else
    {
      // Convert the depth of the central point to millimetres.
      const float depthMm = depth * 1000.0f;

      // This is the definition used in the SCoRe Forests code.
      descriptor.data[depthFeatureOffset + featIdx] = depth1Mm - depthMm;
    }
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
