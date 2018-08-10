/**
 * itmx: ZedImageSourceEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "imagesources/ZedImageSourceEngine.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

ZedImageSourceEngine::ZedImageSourceEngine(const ZedCamera_Ptr& camera)
: m_camera(camera)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMLib::ITMRGBDCalib ZedImageSourceEngine::getCalib() const
{
  return m_camera->get_calib();
}

Vector2i ZedImageSourceEngine::getDepthImageSize() const
{
  return m_camera->get_depth_image_size();
}

void ZedImageSourceEngine::getImages(ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  m_camera->get_images(rgb, rawDepth);
}

Vector2i ZedImageSourceEngine::getRGBImageSize() const
{
  return m_camera->get_rgb_image_size();
}

bool ZedImageSourceEngine::hasImagesNow() const
{
  return m_camera->has_images_now();
}

bool ZedImageSourceEngine::hasMoreImages() const
{
  return m_camera->has_more_images();
}

}
