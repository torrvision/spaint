/**
 * spaint: Fiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/Fiducial.h"

#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

Fiducial::Fiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: m_id(id), m_pose(pose)
{}

//#################### DESTRUCTOR ####################

Fiducial::~Fiducial() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const std::string& Fiducial::id() const
{
  return m_id;
}

const ORUtils::SE3Pose& Fiducial::pose() const
{
  return m_pose;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void Fiducial::check_measurement(const FiducialMeasurement& measurement) const
{
  if(m_id != measurement.id()) throw std::runtime_error("Error: Cannot update a fiducial using a measurement with a different ID");
  if(!measurement.pose_world()) throw std::runtime_error("Error: Cannot update a fiducial using a measurement with no world pose");
}

}
