/**
 * spaint: Fiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/Fiducial.h"

#include <stdexcept>

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

  // Update the derived part of the fiducial.
  integrate_sub(measurement);

  // Update the confidence value for the fiducial.
  const ORUtils::SE3Pose newPose = *measurement.pose_world();
  // TODO
  m_confidence = std::min(m_confidence + 0.1f, 1.0f);
}

const ORUtils::SE3Pose& Fiducial::pose() const
{
  return m_pose;
}

}
