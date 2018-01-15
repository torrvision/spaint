/**
 * itmx: ZedEngine.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "imagesources/ZedEngine.h"

#include <stdexcept>

namespace itmx {

//#################### CONSTRUCTORS ####################

ZedEngine::ZedEngine()
{
  // TODO: Comment here.
  sl::Camera *camera = new sl::Camera;

  // TODO: Comment here.
  sl::InitParameters params;
  params.camera_resolution = sl::RESOLUTION_VGA;
  params.depth_mode = sl::DEPTH_MODE_QUALITY;
  params.coordinate_units = sl::UNIT_METER;

  // TODO: Comment here.
  sl::ERROR_CODE err = camera->open(params);
  if(err == sl::SUCCESS)
  {
    // TODO: Comment here.
    m_camera.reset(camera, destroy_camera);

    // TODO: Comment here.
    m_colourImage.reset(new sl::Mat(m_camera->getResolution(), sl::MAT_TYPE_8U_C4));
    m_depthImage.reset(new sl::Mat);
  }
  else
  {
    // TODO: Comment here.
    delete camera;
    throw std::runtime_error("Error: Could not initialise Zed camera");
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMLib::ITMRGBDCalib ZedEngine::getCalib() const
{
  // TODO
  throw 23;
}

Vector2i ZedEngine::getDepthImageSize() const
{
  return get_image_size();
}

void ZedEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  // TODO: Comment here.
  sl::RuntimeParameters params;
  params.sensing_mode = sl::SENSING_MODE_STANDARD;

  // TODO: Comment here.
  if(m_camera->grab(params) == sl::SUCCESS)
  {
    // TODO: Comment here.
    m_camera->retrieveMeasure(*m_depthImage, sl::MEASURE_DEPTH);
    m_camera->retrieveImage(*m_colourImage, sl::VIEW_LEFT);

    // TODO: Comment here.
    int x;
    x = 23;

    // TODO: Comment here.
    unsigned char *rgbPtr = reinterpret_cast<unsigned char*>(rgb->GetData(MEMORYDEVICE_CPU));
    memcpy(rgbPtr, m_colourImage->getPtr<sl::uchar1>(), m_colourImage->getWidthBytes() * m_colourImage->getHeight());
  }
  else throw std::runtime_error("Error: Could not get images from Zed camera");
}

Vector2i ZedEngine::getRGBImageSize() const
{
  return get_image_size();
}

bool ZedEngine::hasImagesNow() const
{
  return m_camera->isOpened();
}

bool ZedEngine::hasMoreImages() const
{
  return m_camera->isOpened();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

Vector2i ZedEngine::get_image_size() const
{
  sl::Resolution imgSize = m_camera->getResolution();
  return Vector2i(static_cast<int>(imgSize.width), static_cast<int>(imgSize.height));
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void ZedEngine::destroy_camera(sl::Camera *camera)
{
  if(camera)
  {
    camera->close();
    delete camera;
  }
}

}
