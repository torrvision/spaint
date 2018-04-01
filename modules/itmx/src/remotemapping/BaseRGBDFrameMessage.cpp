/**
 * itmx: BaseRGBDFrameMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/BaseRGBDFrameMessage.h"

#include <ORUtils/Math.h>

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

  // Extract both the 4x4 matrix and the rotation/translation vector representations of the pose from the message.
  Matrix4f M = *reinterpret_cast<const Matrix4f*>(&m_data[m_poseSegment.first]);

  float params[6];
  memcpy(params, &m_data[m_poseSegment.first + sizeof(Matrix4f)], 6 * sizeof(float));

  // Check whether or not the rotation/translation vector representation of the pose is valid. Currently, PC clients
  // send both the matrix and the rotation/translation vector representations across to the server to allow it to
  // avoid recalculating the vectors from the matrix (which can cause a pose on the server to be slightly different
  // to that on the client). Android clients currently only send across the matrix, since the vectors are currently
  // not trivially available in the Android application.
  bool vectorsValid = true;
  for(int i = 0; i < 6; ++i)
  {
    // If one of the vectors' components is not equal to itself, it must be NaN, meaning that the vectors are invalid.
    // We avoid using the standard isnan function to check this, since that would force us to depend on C++11.
    if(params[i] != params[i])
    {
      vectorsValid = false;
      break;
    }
  }

  if(vectorsValid) pose.SetBoth(M, params);
  else pose.SetM(M);

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
