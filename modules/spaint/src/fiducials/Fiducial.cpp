/**
 * spaint: Fiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/Fiducial.h"

#include <stdexcept>

#include <ITMLib/Utils/ITMMath.h>

#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

namespace spaint {

//#################### CONSTRUCTORS ####################

Fiducial::Fiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: m_confidence(1.0f), m_id(id), m_pose(pose)
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

  if(GeometryUtil::poses_are_similar(m_pose, *measurement.pose_world()))
  {
    // If the measurement pose is sufficiently similar to the existing fiducial pose, integrate the measurement into the fiducial.
    integrate_sub(measurement);
    ++m_confidence;
  }
  else
  {
    // Otherwise, reset the fiducial pose to the measurement pose, and reset the confidence counter accordingly.
    m_pose = *measurement.pose_world();
    m_confidence = 1.0f;
  }
}

const ORUtils::SE3Pose& Fiducial::pose() const
{
  return m_pose;
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

float Fiducial::stable_confidence()
{
  return 10.0f;
}

}
