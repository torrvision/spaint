/**
 * spaint: Fiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/Fiducial.h"

#include <stdexcept>

#include <ITMLib/Utils/ITMMath.h>

namespace spaint {

//#################### CONSTRUCTORS ####################

Fiducial::Fiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: m_confidence(0.0f), m_id(id), m_pose(pose)
{}

//#################### DESTRUCTOR ####################

Fiducial::~Fiducial() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

float Fiducial::confidence() const
{
  return m_confidence;
}

const std::string& Fiducial::id() const
{
  return m_id;
}

void Fiducial::integrate(const FiducialMeasurement& measurement)
{
  if(m_id != measurement.id()) throw std::runtime_error("Error: Cannot update a fiducial using a measurement with a different ID");
  if(!measurement.pose_world()) throw std::runtime_error("Error: Cannot update a fiducial using a measurement with no world pose");

  Vector3f T, R, newT, newR;
  m_pose.GetParams(T, R);
  measurement.pose_world()->GetParams(newT, newR);
  float dist = length(T - newT);
  float angle = acosf(dot(R.normalised(), newR.normalised()));

  const float distThreshold = 0.05f;
  const float angleThreshold = static_cast<float>(20 * M_PI / 180);
  const float confidenceIncrement = 0.1f;

  if(dist < distThreshold && angle < angleThreshold)
  {
    integrate_sub(measurement);
    m_confidence = std::min(m_confidence + confidenceIncrement, 1.0f);
  }
  else
  {
    m_pose = *measurement.pose_world();
    m_confidence = confidenceIncrement;
  }
}

const ORUtils::SE3Pose& Fiducial::pose() const
{
  return m_pose;
}

}
