/**
 * spaint: SingleRGBDImagePipe.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "imagesources/SingleRGBDImagePipe.h"
using namespace ITMLib;

#include <stdexcept>

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMRGBDCalib SingleRGBDImagePipe::getCalib() const
{
  if(m_calib) return *m_calib;
  else throw std::runtime_error("Error: No calibration parameters available in pipe");
}

Vector2i SingleRGBDImagePipe::getDepthImageSize() const
{
  if(m_rawDepthImage) return m_rawDepthImage->noDims;
  else throw std::runtime_error("Error: No depth image available in pipe");
}

void SingleRGBDImagePipe::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  rawDepth->SetFrom(m_rawDepthImage.get(), ITMShortImage::CPU_TO_CPU);
  rgb->SetFrom(m_rgbImage.get(), ITMUChar4Image::CPU_TO_CPU);

  m_calib.reset();
  m_rawDepthImage.reset();
  m_rgbImage.reset();
}

Vector2i SingleRGBDImagePipe::getRGBImageSize() const
{
  if(m_rgbImage) return m_rgbImage->noDims;
  else throw std::runtime_error("Error: No RGB image available in pipe");
}

bool SingleRGBDImagePipe::hasMoreImages() const
{
  return m_rawDepthImage || m_rgbImage;
}

void SingleRGBDImagePipe::set_calib(const ITMLib::ITMRGBDCalib& calib)
{
  m_calib = calib;
}

void SingleRGBDImagePipe::set_images(const ITMUChar4Image_CPtr& rgbImage, const ITMShortImage_CPtr& rawDepthImage)
{
  m_rawDepthImage = rawDepthImage;
  m_rgbImage = rgbImage;
}

}
