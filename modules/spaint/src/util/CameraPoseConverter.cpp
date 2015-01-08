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

  /*
  Calculate the model-view matrix corresponding to the camera. This should be a matrix that maps
  points from  world XYZ coordinates to camera XYZ coordinates. In practice, our cameras use UVN
  coordinates rather than XYZ coordinates, such that u = -x and v = -y. We will see how to account
  for this shortly.

  The first step is to calculate a matrix mapping points from world XYZ coordinates to camera UVN
  coordinates. This can alternatively be seen as a matrix mapping the camera's UVN axes into the
  world's XYZ axes. As such, it is the inverse of the following matrix, which maps the world's XYZ
  axes into the camera's UVN axes:

  (ux vx nx px)
  (uy vy ny py)
  (uz vz nz pz)
  ( 0  0  0  1)

  This inverse is:

  (ux uy uz -p.u)
  (vx vy vz -p.v)
  (nx ny nz -p.n)
  ( 0  0  0    1)

  This matrix maps points from world XYZ coordinates to camera UVN coordinates, but what we actually
  want is to map to camera XYZ coordinates. To do this, we must premultiply the matrix we have just
  derived by a matrix that transforms from camera UVN coordinates to camera XYZ coordinates, namely:

  (1  0  0 0)
  (0 -1  0 0)
  (0  0 -1 0)
  (0  0  0 1)

  The resulting matrix is then:

  (-ux -uy -uz  p.u)
  (-vx -vy -vz  p.v)
  ( nx  ny  nz -p.n)
  (  0   0   0    1)
  */
  pose.R(0,0) = -u.x(); pose.R(1,0) = -u.y(); pose.R(2,0) = -u.z(); pose.T.x = p.dot(u);
  pose.R(0,1) = -v.x(); pose.R(1,1) = -v.y(); pose.R(2,1) = -v.z(); pose.T.y = p.dot(v);
  pose.R(0,2) = n.x();  pose.R(1,2) = n.y();  pose.R(2,2) = n.z();  pose.T.z = -p.dot(n);

  // Correctly set the rest of the InfiniTAM pose from the model-view matrix.
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
