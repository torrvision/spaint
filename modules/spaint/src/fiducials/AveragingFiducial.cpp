/**
 * spaint: AveragingFiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/AveragingFiducial.h"

#include <iostream>

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/DualQuaternion.h>
using namespace ORUtils;

#include <tvgutil/misc/AttitudeUtil.h>
using namespace tvgutil;

// TEMPORARY
template <typename T>
std::ostream& operator<<(std::ostream& os, const DualNumber<T>& rhs)
{
  os << '(' << rhs.r << ',' << rhs.d << ')';
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const DualQuaternion<T>& rhs)
{
  os << '[' << rhs.w << ',' << rhs.x << ',' << rhs.y << ',' << rhs.z << ']';
  return os;
}
// END TEMPORARY

namespace spaint {

//#################### CONSTRUCTORS ####################

AveragingFiducial::AveragingFiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: Fiducial(id, pose)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void AveragingFiducial::integrate_sub(const FiducialMeasurement& measurement)
{
  // TODO: Comment.
  Vector3f r, t, newR, newT;
  m_pose.GetParams(t, r);
  measurement.pose_world()->GetParams(newT, newR);

  // TODO: Comment.
  DualQuatf q = DualQuatf::from_translation(t) * DualQuatf::from_rotation(r);
  DualQuatf newQ = DualQuatf::from_translation(newT) * DualQuatf::from_rotation(newR);

  // TODO: Calculate the correct interpolation parameter (use the confidence of the fiducial).
  DualQuatf avgQ = DualQuatf::sclerp(newQ, q, m_confidence / (m_confidence + confidence_step()));

  // TODO: Determine avgR and avgT from avgQ.
  // FIXME: Push AttitudeUtil down into InfiniTAM and add a get_rotation member function to DualQuaternion.
  float tempQ[] = { avgQ.w.r, avgQ.x.r, avgQ.y.r, avgQ.z.r };
  float tempR[3];
  try
  {
    AttitudeUtil::quaternion_to_rotation_vector(tempQ, tempR);
  }
  catch(std::runtime_error&)
  {
    std::cout << q << '\n';
    std::cout << newQ << '\n';
    std::cout << m_confidence / (m_confidence + confidence_step()) << '\n';
    std::cout << avgQ << '\n';
    std::cout << tempQ[0] << ' ' << tempQ[1] << ' ' << tempQ[2] << ' ' << tempQ[3] << '\n';
    throw;
  }
  Vector3f avgR(tempR[0], tempR[1], tempR[2]);
  Vector3f avgT = avgQ.get_translation();

  m_pose.SetFrom(avgT, avgR);
}

}
