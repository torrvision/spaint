/**
 * grove: RGBDPatchFeatureCalculator.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "RGBDPatchFeatureCalculator.h"

#include <iostream>

#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

#include <tvgutil/numbers/RandomNumberGenerator.h>
using namespace tvgutil;


//#################### HELPER FUNCTIONS ####################

namespace {

/**
 * \brief Generates a random integer offset in the intervals [-max, -min] and [min, max]
 *        using the specified random number generator.
 *
 * \param rng The random number generator to use.
 * \param min The minimum value of the positive interval.
 * \param max The maximum value of the positive interval.
 * \return    A random integer in [-max, -min] U [min, max].
 */
int generate_offset(RandomNumberGenerator& rng, int min, int max)
{
  static const int signMin = 0;
  static const int signMax = 1;
  return rng.generate_int_from_uniform(std::abs(min), std::abs(max)) * (rng.generate_int_from_uniform(signMin, signMax) * 2 - 1);
}

}

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename KeypointType, typename DescriptorType>
RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::RGBDPatchFeatureCalculator(bool depthAdaptive, uint32_t depthFeatureCount, uint32_t depthFeatureOffset,
                                                                                     uint32_t rgbFeatureCount, uint32_t rgbFeatureOffset)
: m_depthFeatureCount(depthFeatureCount),
  m_depthFeatureOffset(depthFeatureOffset),
  m_featureStep(4), // as per Julien's code (can be overridden with the setter on each invocation)
  m_normaliseDepth(depthAdaptive),
  m_normaliseRgb(depthAdaptive),
  m_rgbFeatureCount(rgbFeatureCount),
  m_rgbFeatureOffset(rgbFeatureOffset)
{
  // Check that the specified feature counts and feature offsets are valid.
  if(depthFeatureCount + rgbFeatureCount > DescriptorType::FEATURE_COUNT)
    throw std::invalid_argument("rgbFeatureCount + depthFeatureCount > DescriptorType::FEATURE_COUNT");
  if(depthFeatureOffset + depthFeatureCount > DescriptorType::FEATURE_COUNT)
    throw std::invalid_argument("depthFeatureOffset + depthFeatureCount > DescriptorType::FEATURE_COUNT");
  if(rgbFeatureOffset + rgbFeatureCount > DescriptorType::FEATURE_COUNT)
    throw std::invalid_argument("rgbFeatureOffset + rgbFeatureCount > DescriptorType::FEATURE_COUNT");

  // Set up the memory blocks.
  const MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
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
void RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::compute_feature(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage, const Vector4f& intrinsics,
                                                                              KeypointsImage *keypointsImage, DescriptorsImage *descriptorsImage) const
{
  // Forward the call on to the version of compute_feature that returns points in world coordinates,
  // passing in the identity matrix as the camera -> world transformation. This will yield keypoints
  // whose 3D coordinates are in the camera's reference frame, as desired.
  Matrix4f identity;
  identity.setIdentity();

  compute_feature(rgbImage, depthImage, identity, intrinsics, keypointsImage, descriptorsImage);
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
void RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::check_input_images(const ITMUChar4Image *rgbImage, const ITMFloatImage *depthImage) const
{
  if((depthImage->noDims.x * depthImage->noDims.y == 0) && (m_normaliseDepth || m_normaliseRgb || m_depthFeatureCount > 0))
  {
    throw std::invalid_argument("Error: A valid depth image is required to compute the features.");
  }

#if 0
  // Note: Since the Structure Sensor does not provide colour information, we do not throw if the colour image is null.
  if((rgbImage->noDims.x * rgbImage->noDims.y == 0) && m_rgbFeatureCount > 0)
  {
    throw std::invalid_argument("Error: A valid colour image is required to compute the features.");
  }
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator<KeypointType,DescriptorType>::setup_colour_features()
{
  // Initialise a random number generator with the default seed found in both the std and boost headers.
  RandomNumberGenerator rng(5489u);

  const int channelMin = 0;
  const int channelMax = 2;
  const int radiusMin = 2;   // as per Julien's code
  const int radiusMax = 130; // as per Julien's code

  Vector4i *offsets = m_rgbOffsets->GetData(MEMORYDEVICE_CPU);
  uchar *channels = m_rgbChannels->GetData(MEMORYDEVICE_CPU);

  for(uint32_t i = 0; i < m_rgbFeatureCount; ++i)
  {
    offsets[i][0] = generate_offset(rng, radiusMin, radiusMax);
    offsets[i][1] = generate_offset(rng, radiusMin, radiusMax);
    offsets[i][2] = generate_offset(rng, radiusMin, radiusMax);
    offsets[i][3] = generate_offset(rng, radiusMin, radiusMax);

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
  RandomNumberGenerator rng(5489u);

  const int radiusMin = 1;       // as per Julien's code (was 2 / 2)
  const int radiusMax = 130 / 2; // as per Julien's code

  Vector4i *offsets = m_depthOffsets->GetData(MEMORYDEVICE_CPU);

  for(uint32_t i = 0; i < m_depthFeatureCount; ++i)
  {
    offsets[i][0] = generate_offset(rng, radiusMin, radiusMax);
    offsets[i][1] = generate_offset(rng, radiusMin, radiusMax);
    offsets[i][2] = generate_offset(rng, radiusMin, radiusMax);
    offsets[i][3] = generate_offset(rng, radiusMin, radiusMax);
  }

#if 0
  // For debugging purposes
  for(uint32_t i = 0; i < m_depthFeatureCount; ++i)
  {
    std::cout << i << " Depth Offset " << offsets[i] << '\n';
  }
#endif
}

}
