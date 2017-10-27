/**
 * itmx: BaseRGBDFrameMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/BaseRGBDFrameMessage.h"

#include <ITMLib/Utils/ITMMath.h>

namespace itmx {

//#################### CONSTRUCTORS ####################

BaseRGBDFrameMessage::BaseRGBDFrameMessage() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int BaseRGBDFrameMessage::extract_frame_index() const
{
  return *reinterpret_cast<const int*>(&m_data[m_frameIndexSegment.first]);
}

ORUtils::SE3Pose BaseRGBDFrameMessage::extract_pose() const
{
  ORUtils::SE3Pose pose;

  Matrix4f M = *reinterpret_cast<const Matrix4f*>(&m_data[m_poseSegment.first]);

  float params[6];
  memcpy(params, &m_data[m_poseSegment.first + sizeof(Matrix4f)], 6 * sizeof(float));

  pose.SetBoth(M, params);
  return pose;
}

void BaseRGBDFrameMessage::set_frame_index(int frameIndex)
{
  memcpy(&m_data[m_frameIndexSegment.first], reinterpret_cast<const char*>(&frameIndex), m_frameIndexSegment.second);
}

void BaseRGBDFrameMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  memcpy(&m_data[m_poseSegment.first], reinterpret_cast<const char*>(&pose.GetM()), sizeof(Matrix4f));
  memcpy(&m_data[m_poseSegment.first + sizeof(Matrix4f)], reinterpret_cast<const char*>(pose.GetParams()), 6 * sizeof(float));
}

}
