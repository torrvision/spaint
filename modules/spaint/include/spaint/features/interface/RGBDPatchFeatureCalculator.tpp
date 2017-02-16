/**
 * spaint: RGBDPatchFeatureCalculator.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "RGBDPatchFeatureCalculator.h"

#include <iostream>

#include <tvgutil/numbers/RandomNumberGenerator.h>
using namespace tvgutil;

#include "util/MemoryBlockFactory.h"

//#################### HELPER FUNCTIONS ####################

namespace
{

/**
 * \brief Generates a random integer offset in the intervals [-max, -min] and [min, max]
 *        using the specified random number generator.
 *
 * \param rng The random number generator to use.
 * \param min TODO
 * \param max TODO
 * \return    TODO
 */
int generate_offset(RandomNumberGenerator& rng, int min, int max)
{
  static const int signMin = 0;
  static const int signMax = 1;
  return rng.generate_int_from_uniform(min, max) * (rng.generate_int_from_uniform(signMin, signMax) * 2 - 1);
}

}

namespace spaint {

//#################### CONSTRUCTORS ####################

template<typename KeypointType, typename DescriptorType>
RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::RGBDPatchFeatureCalculator()
{
  const MemoryBlockFactory& mbf = MemoryBlockFactory::instance();

  // Setup the features in the same way as Julien's code
  m_countDepthFeatures = 128;
  m_countRgbFeatures = 128;
  m_featureStep = 4;
  m_offsetDepthFeatures = 0;
  m_offsetRgbFeatures = m_countDepthFeatures;

  m_normalizeRgb = true;
  m_offsetsRgb = mbf.make_block<Vector4i>(m_countRgbFeatures);
  m_channelsRgb = mbf.make_block<uchar>(m_countRgbFeatures);

  m_normalizeDepth = true;
  m_offsetsDepth = mbf.make_block<Vector4i>(m_countDepthFeatures);

  // Setup colour features
  {
    // Force the default seed found in both the std and boost headers.
    RandomNumberGenerator rng(5489u);

    const int channelMin = 0;
    const int channelMax = 2;
    const int radiusMin = 2;   // From Julien's code.
    const int radiusMax = 130; // From Julien's code.

    Vector4i *offsets = m_offsetsRgb->GetData(MEMORYDEVICE_CPU);
    uchar *channels = m_channelsRgb->GetData(MEMORYDEVICE_CPU);

    for (uint32_t i = 0; i < m_countRgbFeatures; ++i)
    {
      offsets[i][0] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][1] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][2] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][3] = generate_offset(rng, radiusMin, radiusMax);

      // The "2 - rng..." is to perform the RGB2BGR conversion.
      channels[i] = 2 - rng.generate_int_from_uniform(channelMin, channelMax);
    }

//    for (uint32_t i = 0; i < m_countRgbFeatures; ++i)
//    {
//      std::cout << i << " RGB Offset " << offsets[i] << " - Channel: "
//          << channels[i] << std::endl;
//    }
  }

  // Setup depth features
  {
    // Force the default seed found in both the std and boost headers.
    RandomNumberGenerator rng(5489u);

    const int radiusMin = 1;       // From Julien's code (was 2 / 2).
    const int radiusMax = 130 / 2; // From Julien's code.

    Vector4i *offsets = m_offsetsDepth->GetData(MEMORYDEVICE_CPU);

    for (uint32_t i = 0; i < m_countDepthFeatures; ++i)
    {
      offsets[i][0] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][1] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][2] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][3] = generate_offset(rng, radiusMin, radiusMax);
    }

//    for (uint32_t i = 0; i < m_countDepthFeatures; ++i)
//    {
//      std::cout << i << " Depth Offset " << offsets[i] << std::endl;
//    }
  }
}

//#################### DESTRUCTOR ####################

template<typename KeypointType, typename DescriptorType>
RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::~RGBDPatchFeatureCalculator() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template<typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::compute_feature(const ITMUChar4Image *rgbImage,
    const ITMFloatImage *depthImage, const Vector4f &intrinsics, KeypointImage *keypointsImage,
    DescriptorImage *featuresImage) const
{
  // Use the identity transform to call the virtual function.
  // Keypoints will have 3D coordinates in camera reference frame.
  Matrix4f identity;
  identity.setIdentity();

  compute_feature(rgbImage, depthImage, intrinsics, keypointsImage,
      featuresImage, identity);
}

template<typename KeypointType, typename DescriptorType>
uint32_t RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::get_feature_step() const
{
  return m_featureStep;
}

template<typename KeypointType, typename DescriptorType>
void RGBDPatchFeatureCalculator<KeypointType, DescriptorType>::set_feature_step(uint32_t featureStep)
{
  m_featureStep = featureStep;
}

}
