/**
 * itmx: SingleRGBDImagePipe.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "imagesources/SingleRGBDImagePipe.h"
using namespace ITMLib;

#include <stdexcept>

namespace itmx {

//#################### CONSTRUCTORS ####################

SingleRGBDImagePipe::SingleRGBDImagePipe(const ImageSourceEngine_CPtr& imageSourceEngine)
: m_calib(imageSourceEngine->getCalib()),
  m_depthImageSize(imageSourceEngine->getDepthImageSize()),
  m_rgbImageSize(imageSourceEngine->getRGBImageSize())
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMRGBDCalib SingleRGBDImagePipe::getCalib() const
{
  return m_calib;
}

Vector2i SingleRGBDImagePipe::getDepthImageSize() const
{
  return m_depthImageSize;
}

void SingleRGBDImagePipe::getImages(ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  rawDepth->SetFrom(m_depthImage.get(), ORShortImage::CPU_TO_CPU);
  rgb->SetFrom(m_rgbImage.get(), ORUChar4Image::CPU_TO_CPU);

  m_depthImage.reset();
  m_rgbImage.reset();
}

Vector2i SingleRGBDImagePipe::getRGBImageSize() const
{
  return m_rgbImageSize;
}

bool SingleRGBDImagePipe::hasMoreImages() const
{
  return m_depthImage || m_rgbImage;
}

void SingleRGBDImagePipe::set_images(const ORUChar4Image_CPtr& rgbImage, const ORShortImage_CPtr& depthImage)
{
  m_depthImage = depthImage;
  m_rgbImage = rgbImage;
}

}
