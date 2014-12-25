/**
 * spaint: CameraPoseConverter.cpp
 */

#include "util/CameraPoseConverter.h"
using namespace ITMLib::Objects;
using namespace rigging;

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ITMPose CameraPoseConverter::camera_to_pose(const Camera& camera)
{
  ITMPose pose;

  const Eigen::Vector3f& n = camera.n();
  const Eigen::Vector3f& p = camera.p();
  const Eigen::Vector3f& u = camera.u();
  const Eigen::Vector3f& v = camera.v();

  // Calculate the model-view matrix corresponding to the camera.
  pose.R(0,0) = -u.x(); pose.R(1,0) = -u.y(); pose.R(2,0) = -u.z(); pose.T.x = p.dot(u);
  pose.R(0,1) = -v.x(); pose.R(1,1) = -v.y(); pose.R(2,1) = -v.z(); pose.T.y = p.dot(v);
  pose.R(0,2) = n.x();  pose.R(1,2) = n.y();  pose.R(2,2) = n.z();  pose.T.z = -p.dot(n);

  // Determine the InfiniTAM pose from the model-view matrix.
  pose.SetParamsFromModelView();
  pose.SetModelViewFromParams();

  return pose;
}

SimpleCamera CameraPoseConverter::pose_to_camera(const ITMLib::Objects::ITMPose& pose)
{
  // Note: This can be derived by looking at the matrix in camera_to_pose.
  Eigen::Vector3f p(pose.invT.x, pose.invT.y, pose.invT.z);
  Eigen::Vector3f n(pose.R(0,2), pose.R(1,2), pose.R(2,2));
  Eigen::Vector3f v(-pose.R(0,1), -pose.R(1,1), -pose.R(2,1));
  return SimpleCamera(p, n, v);
}

}
