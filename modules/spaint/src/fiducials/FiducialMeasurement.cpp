/**
 * spaint: FiducialMeasurement.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/FiducialMeasurement.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

FiducialMeasurement::FiducialMeasurement(const std::string& id, const boost::optional<ORUtils::SE3Pose>& poseEye, const boost::optional<ORUtils::SE3Pose>& poseWorld)
: m_id(id), m_poseEye(poseEye), m_poseWorld(poseWorld)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const std::string& FiducialMeasurement::id() const
{
  return m_id;
}

const boost::optional<ORUtils::SE3Pose>& FiducialMeasurement::pose_eye() const
{
  return m_poseEye;
}


const boost::optional<ORUtils::SE3Pose>& FiducialMeasurement::pose_world() const
{
  return m_poseWorld;
}

}
