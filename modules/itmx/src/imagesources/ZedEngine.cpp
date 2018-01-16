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
  sl::InitParameters initParams;
  initParams.camera_resolution = sl::RESOLUTION_VGA;
  initParams.coordinate_units = sl::UNIT_METER;
  initParams.depth_mode = sl::DEPTH_MODE_PERFORMANCE;

  initParams.sdk_gpu_id = 0;
  cuCtxGetCurrent(&initParams.sdk_cuda_ctx);

  // TODO: Comment here.
  sl::ERROR_CODE err = camera->open(initParams);
  if(err != sl::SUCCESS)
  {
    delete camera;
    return;
  }

  // TODO: Comment here.
  m_camera.reset(camera, destroy_camera);

  // TODO: Comment here.
  m_colourImage.reset(new sl::Mat(m_camera->getResolution(), sl::MAT_TYPE_8U_C4));
  m_depthImage.reset(new sl::Mat);

  // TODO: Comment here.
  sl::CalibrationParameters calibParams = m_camera->getCameraInformation().calibration_parameters;
  m_calib.intrinsics_rgb.SetFrom(calibParams.left_cam.fx, calibParams.left_cam.fy, calibParams.left_cam.cx, calibParams.left_cam.cy);
  m_calib.intrinsics_d = m_calib.intrinsics_rgb;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMLib::ITMRGBDCalib ZedEngine::getCalib() const
{
  return m_calib;
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
    m_camera->retrieveImage(*m_colourImage, sl::VIEW_LEFT);
    m_camera->retrieveMeasure(*m_depthImage, sl::MEASURE_DEPTH);

    // TODO: Comment here.
    {
      unsigned char *dest = reinterpret_cast<unsigned char*>(rgb->GetData(MEMORYDEVICE_CPU));
      const unsigned char *src = m_colourImage->getPtr<sl::uchar1>();
      for(size_t i = 0, size = m_colourImage->getWidthBytes() * m_colourImage->getHeight(); i < size; i += 4)
      {
        // Convert BGRA to RGBA.
        dest[i+0] = src[i+2];
        dest[i+1] = src[i+1];
        dest[i+2] = src[i+0];
        dest[i+3] = src[i+3];
      }
    }

    // TODO: Comment here.
    {
      short *dest = rawDepth->GetData(MEMORYDEVICE_CPU);
      const float *src = m_depthImage->getPtr<float>();
      for(size_t i = 0, size = m_depthImage->getWidth() * m_depthImage->getHeight(); i < size; ++i)
      {
        dest[i] = (short)(CLAMP(ROUND(src[i] / m_calib.disparityCalib.GetParams()[0]), 0, std::numeric_limits<short>::max()));
      }
    }
  }
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
  if(m_camera)
  {
    sl::Resolution imgSize = m_camera->getResolution();
    return Vector2i(static_cast<int>(imgSize.width), static_cast<int>(imgSize.height));
  }
  else return Vector2i(0,0);
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
