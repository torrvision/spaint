/**
 * spaint: RGBDPatchFeatureCalculator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "features/interface/RGBDPatchFeatureCalculator.h"

#include <iostream>

#include "util/MemoryBlockFactory.h"

#include "tvgutil/numbers/RandomNumberGenerator.h"

namespace spaint
{

namespace
{
/**
 * \brief Generates a random integer offset in the intervals [-max, -min] and [min, max] using rng.
 */
int generate_offset(tvgutil::RandomNumberGenerator &rng, int min, int max)
{
  static const int signMin = 0;
  static const int signMax = 1;
  return rng.generate_int_from_uniform(min, max)
      * (rng.generate_int_from_uniform(signMin, signMax) * 2 - 1);
}
}

RGBDPatchFeatureCalculator::RGBDPatchFeatureCalculator()
{
  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  // Setup the features in the same way as Julien's code
  m_featureStep = 4;

  m_normalizeRgb = true;
  m_offsetsRgb = mbf.make_block<Vector4i>(
      RGBDPatchDescriptor::RGB_FEATURE_COUNT);
  m_channelsRgb = mbf.make_block<uchar>(RGBDPatchDescriptor::RGB_FEATURE_COUNT);

  m_normalizeDepth = true;
  m_offsetsDepth = mbf.make_block<Vector4i>(
      RGBDPatchDescriptor::DEPTH_FEATURE_COUNT);

  // Setup colour features
  {
    // Force the default seed found in both the std and boost headers.
    tvgutil::RandomNumberGenerator rng(5489u);

    const int channelMin = 0;
    const int channelMax = 2;
    const int radiusMin = 2;
    const int radiusMax = 130;

    Vector4i *offsets = m_offsetsRgb->GetData(MEMORYDEVICE_CPU);
    uchar *channels = m_channelsRgb->GetData(MEMORYDEVICE_CPU);

    for (int i = 0; i < RGBDPatchDescriptor::RGB_FEATURE_COUNT; ++i)
    {
      offsets[i][0] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][1] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][2] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][3] = generate_offset(rng, radiusMin, radiusMax);

      // RGB2BGR
      channels[i] = 2 - rng.generate_int_from_uniform(channelMin, channelMax);
    }

//    for (int i = 0; i < RGBDPatchDescriptor::RGB_FEATURE_COUNT; ++i)
//    {
//      std::cout << i << " RGB Offset " << offsets[i] << " - Channel: "
//          << channels[i] << std::endl;
//    }
  }

  // Setup depth features
  {
    // Force the default seed found in both the std and boost headers.
    tvgutil::RandomNumberGenerator rng(5489u);

    const int radiusMin = 2 / 2;
    const int radiusMax = 130 / 2;

    Vector4i *offsets = m_offsetsDepth->GetData(MEMORYDEVICE_CPU);

    for (int i = 0; i < RGBDPatchDescriptor::DEPTH_FEATURE_COUNT; ++i)
    {
      offsets[i][0] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][1] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][2] = generate_offset(rng, radiusMin, radiusMax);
      offsets[i][3] = generate_offset(rng, radiusMin, radiusMax);
    }

//    for (int i = 0; i < RGBDPatchDescriptor::DEPTH_FEATURE_COUNT; ++i)
//    {
//      std::cout << i << " Depth Offset " << offsets[i] << std::endl;
//    }
  }
}

RGBDPatchFeatureCalculator::~RGBDPatchFeatureCalculator()
{
}

void RGBDPatchFeatureCalculator::compute_feature(const ITMUChar4Image *rgbImage,
    const ITMFloatImage *depthImage, const Vector4f &intrinsics,
    Keypoint3DColourImage *keypointsImage,
    RGBDPatchDescriptorImage *featuresImage) const
{
  Matrix4f identity;
  identity.setIdentity();

  compute_feature(rgbImage, depthImage, intrinsics, keypointsImage,
      featuresImage, identity);
}

void RGBDPatchFeatureCalculator::set_feature_step(uint32_t featureStep)
{
  m_featureStep = featureStep;
}

uint32_t RGBDPatchFeatureCalculator::get_feature_step() const
{
  return m_featureStep;
}

}
