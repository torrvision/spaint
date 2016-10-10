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
: m_confidence(confidence_increment()), m_id(id), m_pose(pose)
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

  Vector3f R, t, newR, newT;
  m_pose.GetParams(t, R);
  measurement.pose_world()->GetParams(newT, newR);
  float dist = length(t - newT);
  float angle = acosf(dot(R.normalised(), newR.normalised()));

  const float distThreshold = 0.05f;
  const float angleThreshold = static_cast<float>(20 * M_PI / 180);

  if(dist < distThreshold && angle < angleThreshold)
  {
    integrate_sub(measurement);
    m_confidence = std::min(m_confidence + confidence_increment(), 1.0f);
  }
  else
  {
    m_pose = *measurement.pose_world();
    m_confidence = confidence_increment();
  }
}

const ORUtils::SE3Pose& Fiducial::pose() const
{
  return m_pose;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

float Fiducial::confidence_increment()
{
  return 0.1f;
}

}
