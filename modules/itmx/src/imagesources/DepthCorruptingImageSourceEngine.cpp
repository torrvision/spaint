/**
 * itmx: DepthCorruptingImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#include "imagesources/DepthCorruptingImageSourceEngine.h"

#include <ITMLib/Engines/ViewBuilding/Shared/ITMViewBuilder_Shared.h>
using namespace ITMLib;

namespace itmx {

//#################### CONSTRUCTORS ####################

DepthCorruptingImageSourceEngine::DepthCorruptingImageSourceEngine(ImageSourceEngine *innerSource, double missingDepthFraction, float depthNoiseSigma)
: m_depthNoiseSigma(depthNoiseSigma), m_innerSource(innerSource), m_rng(12345)
{
  if(missingDepthFraction > 0.0)
  {
    m_missingDepthMask.reset(new ORBoolImage(innerSource->getDepthImageSize(), true, false));
    bool *missingDepthMask = m_missingDepthMask->GetData(MEMORYDEVICE_CPU);
    for(size_t i = 0, size = m_missingDepthMask->dataSize; i < size; ++i)
    {
      missingDepthMask[i] = m_rng.generate_real_from_uniform<double>(0.0, 1.0) <= missingDepthFraction;
    }
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMRGBDCalib DepthCorruptingImageSourceEngine::getCalib() const
{
  return m_innerSource->getCalib();
}

Vector2i DepthCorruptingImageSourceEngine::getDepthImageSize() const
{
  return m_innerSource->getDepthImageSize();
}

void DepthCorruptingImageSourceEngine::getImages(ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  const Vector2f depthCalibParams = getCalib().disparityCalib.GetParams();

  // Get the uncorrupted images.
  m_innerSource->getImages(rgb, rawDepth);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int y = 0; y < rawDepth->noDims.y; ++y)
  {
    for(int x = 0; x < rawDepth->noDims.x; ++x)
    {
      const int offset = y * rawDepth->noDims.x + x;
      short& rawDepthValue = rawDepth->GetData(MEMORYDEVICE_CPU)[offset];

      // If desired, zero out any depth values that should be missing.
      if(m_missingDepthMask && m_missingDepthMask->GetData(MEMORYDEVICE_CPU)[offset])
      {
        rawDepthValue = 0;
      }

      // If desired, corrupt any depth values that still exist with zero-mean, depth-dependent Gaussian noise.
      if(m_depthNoiseSigma > 0.0f && rawDepthValue != 0)
      {
        float depth = 0.0f;
        convertDepthAffineToFloat(&depth, 0, 0, &rawDepthValue, rawDepth->noDims, depthCalibParams);

      #ifdef WITH_OPENMP
        #pragma omp critical
      #endif
        depth += m_rng.generate_from_gaussian(0.0f, m_depthNoiseSigma) * depth;

        rawDepthValue = CLAMP(static_cast<short>(ROUND((depth - depthCalibParams.y) / depthCalibParams.x)), 1, 32000);
      }
    }
  }

  rawDepth->UpdateDeviceFromHost();
}

Vector2i DepthCorruptingImageSourceEngine::getRGBImageSize() const
{
  return m_innerSource->getRGBImageSize();
}

bool DepthCorruptingImageSourceEngine::hasMoreImages() const
{
  return m_innerSource->hasMoreImages();
}

}
