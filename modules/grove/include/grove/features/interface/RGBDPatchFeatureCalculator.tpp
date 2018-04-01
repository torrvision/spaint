/**
 * grove: RGBDPatchFeatureCalculator.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "RGBDPatchFeatureCalculator.h"

#include <iostream>

#include <itmx/base/MemoryBlockFactory.h>

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename KeypointType, typename DescriptorType>
RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::RGBDPatchFeatureCalculator(
  bool depthAdaptive, RGBDPatchFeatureDifferenceType depthDifferenceType,
  uint32_t depthFeatureCount, uint32_t depthFeatureOffset, uint32_t depthMinRadius,
  uint32_t depthMaxRadius, RGBDPatchFeatureDifferenceType rgbDifferenceType,
  uint32_t rgbFeatureCount, uint32_t rgbFeatureOffset, uint32_t rgbMinRadius, uint32_t rgbMaxRadius
)
: m_depthDifferenceType(depthDifferenceType),
  m_depthFeatureCount(depthFeatureCount),
  m_depthFeatureOffset(depthFeatureOffset),
  m_depthMaxRadius(depthMaxRadius),
  m_depthMinRadius(depthMinRadius),
  m_featureStep(4), // as per Julien's code (can be overridden with the setter)
  m_normaliseDepth(depthAdaptive),
  m_normaliseRgb(depthAdaptive),
  m_rgbDifferenceType(rgbDifferenceType),
  m_rgbFeatureCount(rgbFeatureCount),
  m_rgbFeatureOffset(rgbFeatureOffset),
  m_rgbMaxRadius(rgbMaxRadius),
  m_rgbMinRadius(rgbMinRadius)
{
  // Check that the specified feature counts and feature offsets are valid.
  if(depthFeatureCount + rgbFeatureCount > DescriptorType::FEATURE_COUNT)
    throw std::invalid_argument("rgbFeatureCount + depthFeatureCount > DescriptorType::FEATURE_COUNT");
  if(depthFeatureOffset + depthFeatureCount > DescriptorType::FEATURE_COUNT)
    throw std::invalid_argument("depthFeatureOffset + depthFeatureCount > DescriptorType::FEATURE_COUNT");
  if(rgbFeatureOffset + rgbFeatureCount > DescriptorType::FEATURE_COUNT)
    throw std::invalid_argument("rgbFeatureOffset + rgbFeatureCount > DescriptorType::FEATURE_COUNT");

  // Set up the memory blocks used to specify the features.
  const itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();
  m_depthOffsets = mbf.make_block<Vector4i>(m_depthFeatureCount);
  m_rgbChannels = mbf.make_block<uchar>(m_rgbFeatureCount);
  m_rgbOffsets = mbf.make_block<Vector4i>(m_rgbFeatureCount);

  // Set up the features.
  setup_depth_features();
  setup_colour_features();
}

//#################### DESTRUCTOR ####################

template <typename KeypointType, typename DescriptorType>
RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::~RGBDPatchFeatureCalculator()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::compute_keypoints_and_features(const ORUChar4Image *rgbImage, const ORFloatImage *depthImage, const Vector4f& intrinsics,
                                                                                             KeypointsImage *keypointsImage, DescriptorsImage *descriptorsImage) const
{
  // Forward the call on to the version of compute_keypoints_and_features that returns points in world coordinates,
  // passing in the identity matrix as the camera -> world transformation. This will yield keypoints whose 3D
  // coordinates are in the camera's reference frame, as desired.
  Matrix4f identity;
  identity.setIdentity();

  compute_keypoints_and_features(rgbImage, depthImage, identity, intrinsics, keypointsImage, descriptorsImage);
}

template <typename KeypointType, typename DescriptorType>
uint32_t RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::get_feature_step() const
{
  return m_featureStep;
}

template <typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::set_feature_step(uint32_t featureStep)
{
  m_featureStep = featureStep;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

template <typename KeypointType, typename DescriptorType>
Vector2i RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::compute_output_dims(const ORUChar4Image *rgbImage, const ORFloatImage *depthImage) const
{
  const bool requireColour = m_rgbFeatureCount > 0;
  const bool requireDepth = m_normaliseDepth || m_normaliseRgb || m_depthFeatureCount > 0;

  const bool validColour = rgbImage && (rgbImage->noDims.x * rgbImage->noDims.y > 0);
  const bool validDepth = depthImage && (depthImage->noDims.x * depthImage->noDims.y > 0);

  if(requireDepth && !validDepth)
  {
    throw std::invalid_argument("Error: A valid depth image is required to compute the features.");
  }

  // Note: Since the Structure Sensor does not provide colour information, we only throw
  //       if the colour image is NULL and we are not computing depth features.
  if(requireColour && !validColour && !requireDepth)
  {
    throw std::invalid_argument("Error: A valid colour image is required to compute the features.");
  }

  // Use the depth image size as base, unless we want colour features only.
  Vector2i inputDims = (requireDepth && validDepth) ? depthImage->noDims : rgbImage->noDims;

  // Compute the output dimensions.
  return inputDims / m_featureStep;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::setup_colour_features()
{
  // Initialise a random number generator with the default seed found in both the std and boost headers.
  tvgutil::RandomNumberGenerator rng(5489u);

  const int channelMin = 0;
  const int channelMax = 2;

  Vector4i *offsets = m_rgbOffsets->GetData(MEMORYDEVICE_CPU);
  uchar *channels = m_rgbChannels->GetData(MEMORYDEVICE_CPU);

  for(uint32_t i = 0; i < m_rgbFeatureCount; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      // Note: The range of the offsets used is designed to be consistent with the range of the depths by which we might later normalise them.
      offsets[i][j] = generate_offset(rng, m_rgbMinRadius, m_rgbMaxRadius);
    }

    // The "2 - rng..." is to perform the RGB to BGR conversion.
    channels[i] = 2 - rng.generate_int_from_uniform(channelMin, channelMax);
  }

#if 0
  // For debugging purposes
  for(uint32_t i = 0; i < m_rgbFeatureCount; ++i)
  {
    std::cout << i << " RGB Offset " << offsets[i] << " - Channel: " << channels[i] << '\n';
  }
#endif
}

template <typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::setup_depth_features()
{
  // Initialise a random number generator with the default seed found in both the std and boost headers.
  tvgutil::RandomNumberGenerator rng(5489u);

  Vector4i *offsets = m_depthOffsets->GetData(MEMORYDEVICE_CPU);

  for(uint32_t i = 0; i < m_depthFeatureCount; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      offsets[i][j] = generate_offset(rng, m_depthMinRadius, m_depthMaxRadius);
    }
  }

#if 0
  // For debugging purposes
  for(uint32_t i = 0; i < m_depthFeatureCount; ++i)
  {
    std::cout << i << " Depth Offset " << offsets[i] << '\n';
  }
#endif
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

template <typename KeypointType, typename DescriptorType>
int RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::generate_offset(tvgutil::RandomNumberGenerator& rng, int min, int max)
{
  static const int signMin = 0;
  static const int signMax = 1;
  return rng.generate_int_from_uniform(std::abs(min), std::abs(max)) * (rng.generate_int_from_uniform(signMin, signMax) * 2 - 1);
}

}
