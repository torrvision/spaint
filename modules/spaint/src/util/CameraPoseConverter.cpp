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
  Calculate the InfiniTAM pose matrix corresponding to the camera. This is the matrix that transforms
  points in the space corresponding to the initial pose of the camera in InfiniTAM (n = (0,0,1)^T,
  u = (-1,0,0)^T, v = (0,-1,0)^T, p = (0,0,0)^T) into the space corresponding to the current pose
  of the camera. (Alternatively, it can be seen as the matrix that maps the current camera axes to
  the initial camera axes when both are expressed in world coordinates.)

  Notationally, let I be the initial camera space, C be the current camera space and W be the world
  space. Furthermore, let bTa denote the transformation that maps points from space a to space b.
  Thus cTi denotes the pose matrix, namely the matrix that transforms points from initial camera
  space to current camera space, and cTw = cTi . iTw denotes the model-view matrix that would be
  used in OpenGL. The matrix cTw is easy to calculate, and can be used to calculate the required
  pose matrix via cTi = cTw . iTw^-1 = cTw . wTi.

  To calculate cTw, the matrix that maps world space points to camera space ones, first calculate
  its inverse wTc, the matrix that maps camera space points to world space ones. This is trivial:

  (ux vx nx px)
  (uy vy ny py)
  (uz vz nz pz)
  ( 0  0  0  1)

  From this we can obtain cTw:

  (ux uy uz -p.u)
  (vx vy vz -p.v)
  (nx ny nz -p.n)
  ( 0  0  0    1)

  What is wTi, the matrix mapping points in initial camera space to points in world space? Well,
  bearing in mind the initial pose of the camera, it must be:

  (-1  0 0 0)
  ( 0 -1 0 0)
  ( 0  0 1 0)
  ( 0  0 0 1)

  We can thus easily obtain the required pose matrix cTi by matrix multiplication:

  (-ux -uy -uz  p.u)
  (-vx -vy -vz  p.v)
  ( nx  ny  nz -p.n)
  (  0   0   0    1)
  */
  pose.R(0,0) = -u.x(); pose.R(1,0) = -u.y(); pose.R(2,0) = -u.z(); pose.T.x = p.dot(u);
  pose.R(0,1) = -v.x(); pose.R(1,1) = -v.y(); pose.R(2,1) = -v.z(); pose.T.y = p.dot(v);
  pose.R(0,2) = n.x();  pose.R(1,2) = n.y();  pose.R(2,2) = n.z();  pose.T.z = -p.dot(n);

  /*
  Correctly set the rest of the InfiniTAM pose structure from the calculated pose matrix.
  Note that the use of "ModelView" in the names of the ITMPose member functions refers
  to the InfiniTAM pose matrix, and NOT the OpenGL model-view matrix. This is a crucial
  distinction that can cause a lot of confusion!
  */
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
