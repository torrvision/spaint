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
  m_depthCompressionTypeSegment = std::make_pair(0, sizeof(DepthCompressionType));
  m_rgbCompressionTypeSegment = std::make_pair(m_depthCompressionTypeSegment.first + m_depthCompressionTypeSegment.second, sizeof(RGBCompressionType));
  m_calibSegment = std::make_pair(
    m_rgbCompressionTypeSegment.first + m_rgbCompressionTypeSegment.second,
    sizeof(Vector2f) + sizeof(ITMDisparityCalib::TrafoType) + // disparityCalib
    sizeof(Vector2i) + sizeof(Vector4f) +                     // intrinsics_d
    sizeof(Vector2i) + sizeof(Vector4f) +                     // intrinsics_rgb
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

  Vector2i imgSize = *reinterpret_cast<const Vector2i*>(p);
  p += sizeof(Vector2i);
  Vector4f projectionParams = *reinterpret_cast<const Vector4f*>(p);
  p += sizeof(Vector4f);
  calib.intrinsics_d.SetFrom(imgSize.x, imgSize.y, projectionParams[0], projectionParams[1], projectionParams[2], projectionParams[3]);

  imgSize = *reinterpret_cast<const Vector2i*>(p);
  p += sizeof(Vector2i);
  projectionParams = *reinterpret_cast<const Vector4f*>(p);
  p += sizeof(Vector4f);
  calib.intrinsics_rgb.SetFrom(imgSize.x, imgSize.y, projectionParams[0], projectionParams[1], projectionParams[2], projectionParams[3]);

  const Matrix4f m = *reinterpret_cast<const Matrix4f*>(p);
  calib.trafo_rgb_to_depth.SetFrom(m);

  return calib;
}

DepthCompressionType RGBDCalibrationMessage::extract_depth_compression_type() const
{
  return *reinterpret_cast<const DepthCompressionType*>(&m_data[m_depthCompressionTypeSegment.first]);
}

RGBCompressionType RGBDCalibrationMessage::extract_rgb_compression_type() const
{
  return *reinterpret_cast<const RGBCompressionType*>(&m_data[m_rgbCompressionTypeSegment.first]);
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

  memcpy(p, reinterpret_cast<const char*>(&calib.intrinsics_d.imgSize), sizeof(Vector2i));
  p += sizeof(Vector2i);
  memcpy(p, reinterpret_cast<const char*>(&calib.intrinsics_d.projectionParamsSimple.all), sizeof(Vector4f));
  p += sizeof(Vector4f);

  memcpy(p, reinterpret_cast<const char*>(&calib.intrinsics_rgb.imgSize), sizeof(Vector2i));
  p += sizeof(Vector2i);
  memcpy(p, reinterpret_cast<const char*>(&calib.intrinsics_rgb.projectionParamsSimple.all), sizeof(Vector4f));
  p += sizeof(Vector4f);

  memcpy(p, reinterpret_cast<const char*>(&calib.trafo_rgb_to_depth.calib), sizeof(Matrix4f));
}

void RGBDCalibrationMessage::set_depth_compression_type(DepthCompressionType depthCompressionType)
{
  memcpy(&m_data[m_depthCompressionTypeSegment.first], reinterpret_cast<const char*>(&depthCompressionType), m_depthCompressionTypeSegment.second);
}

void RGBDCalibrationMessage::set_rgb_compression_type(RGBCompressionType rgbCompressionType)
{
  memcpy(&m_data[m_rgbCompressionTypeSegment.first], reinterpret_cast<const char*>(&rgbCompressionType), m_rgbCompressionTypeSegment.second);
}

}
