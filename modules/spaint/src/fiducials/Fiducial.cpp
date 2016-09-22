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

//#################### PUBLIC MEMBER FUNCTIONS ####################

const std::string& Fiducial::id() const
{
  return m_id;
}

const ORUtils::SE3Pose& Fiducial::pose() const
{
  return m_pose;
}

void Fiducial::update(const Fiducial& newFiducial)
{
  if(m_id != newFiducial.m_id)
  {
    throw std::runtime_error("Error: Cannot update a fiducial using a fiducial with a different ID");
  }

  // For now, just overwrite the properties of this fiducial with those of the new fiducial.
  // (More sophisticated alternatives can be implemented later if necessary.)
  m_pose = newFiducial.m_pose;
}

}
