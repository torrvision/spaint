/**
 * itmx: RGBDCalibrationMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/RGBDCalibrationMessage.h"
using namespace ITMLib;

namespace itmx {

//#################### CONSTRUCTORS ####################

RGBDCalibrationMessage::RGBDCalibrationMessage()
{
  m_rgbImageSizeSegment = std::make_pair(0, sizeof(Vector2i));
  m_depthImageSizeSegment = std::make_pair(m_rgbImageSizeSegment.second, sizeof(Vector2i));
  m_calibSegment = std::make_pair(
    m_depthImageSizeSegment.first + m_depthImageSizeSegment.second,
    sizeof(Vector2f) + sizeof(ITMDisparityCalib::TrafoType) + // disparityCalib
    sizeof(Vector4f) +                                        // intrinsics_d
    sizeof(Vector4f) +                                        // intrinsics_rgb
    sizeof(Matrix4f)                                          // trafo_rgb_to_depth
  );
  m_data.resize(m_calibSegment.first + m_calibSegment.second);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMRGBDCalib RGBDCalibrationMessage::extract_calib() const
{
  ITMRGBDCalib calib;

  const char *p = &m_data[m_calibSegment.first];

  const Vector2f params = *reinterpret_cast<const Vector2f*>(p);
  p += sizeof(Vector2f);

  const ITMDisparityCalib::TrafoType type = *reinterpret_cast<const ITMDisparityCalib::TrafoType*>(p);
  p += sizeof(ITMDisparityCalib::TrafoType);

  calib.disparityCalib.SetFrom(params.x, params.y, type);

  calib.intrinsics_d.projectionParamsSimple.all = *reinterpret_cast<const Vector4f*>(p);
  p += sizeof(Vector4f);

  calib.intrinsics_rgb.projectionParamsSimple.all = *reinterpret_cast<const Vector4f*>(p);
  p += sizeof(Vector4f);

  const Matrix4f m = *reinterpret_cast<const Matrix4f*>(p);
  calib.trafo_rgb_to_depth.SetFrom(m);

  return calib;
}

Vector2i RGBDCalibrationMessage::extract_depth_image_size() const
{
  return *reinterpret_cast<const Vector2i*>(&m_data[m_depthImageSizeSegment.first]);
}

Vector2i RGBDCalibrationMessage::extract_rgb_image_size() const
{
  return *reinterpret_cast<const Vector2i*>(&m_data[m_rgbImageSizeSegment.first]);
}

void RGBDCalibrationMessage::set_calib(const ITMRGBDCalib& calib)
{
  char *p = &m_data[m_calibSegment.first];

  const Vector2f params = calib.disparityCalib.GetParams();
  memcpy(p, reinterpret_cast<const char*>(&params), sizeof(Vector2f));
  p += sizeof(Vector2f);

  const ITMDisparityCalib::TrafoType type = calib.disparityCalib.GetType();
  memcpy(p, reinterpret_cast<const char*>(&type), sizeof(ITMDisparityCalib::TrafoType));
  p += sizeof(ITMDisparityCalib::TrafoType);

  memcpy(p, reinterpret_cast<const char*>(&calib.intrinsics_d.projectionParamsSimple.all), sizeof(Vector4f));
  p += sizeof(Vector4f);

  memcpy(p, reinterpret_cast<const char*>(&calib.intrinsics_rgb.projectionParamsSimple.all), sizeof(Vector4f));
  p += sizeof(Vector4f);

  memcpy(p, reinterpret_cast<const char*>(&calib.trafo_rgb_to_depth.calib), sizeof(Matrix4f));
}

void RGBDCalibrationMessage::set_depth_image_size(const Vector2i& depthImageSize)
{
  memcpy(&m_data[m_depthImageSizeSegment.first], reinterpret_cast<const char*>(&depthImageSize), m_depthImageSizeSegment.second);
}

void RGBDCalibrationMessage::set_rgb_image_size(const Vector2i& rgbImageSize)
{
  memcpy(&m_data[m_rgbImageSizeSegment.first], reinterpret_cast<const char*>(&rgbImageSize), m_rgbImageSizeSegment.second);
}

}
