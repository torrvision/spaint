/**
 * rigging: MoveableCamera.h
 */

#ifndef H_RIGGING_MOVEABLECAMERA
#define H_RIGGING_MOVEABLECAMERA

#include "Camera.h"

namespace rigging {

/**
 * \brief An instance of a class deriving from this one represents a moveable camera in 3D space.
 */
class MoveableCamera : public Camera
{
  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Moves the camera by the specified displacement in the n direction.
   *
   * \param delta The displacement by which to move.
   * \return      This camera, after it has been moved.
   */
  virtual MoveableCamera& move_n(float delta) = 0;

  /**
   * \brief Moves the camera by the specified displacement in the u direction.
   *
   * \param delta The displacement by which to move.
   * \return      This camera, after it has been moved.
   */
  virtual MoveableCamera& move_u(float delta) = 0;

  /**
   * \brief Moves the camera by the specified displacement in the v direction.
   *
   * \param delta The displacement by which to move.
   * \return      This camera, after it has been moved.
   */
  virtual MoveableCamera& move_v(float delta) = 0;

  /**
   * \brief Rotates the camera anticlockwise by the specified angle about the specified axis.
   *
   * \param axis  The axis about which to rotate.
   * \param angle The angle by which to rotate (in radians).
   * \return      This camera, after it has been rotated.
   */
  virtual MoveableCamera& rotate(const Eigen::Vector3f& axis, float angle) = 0;

  /**
   * \brief Sets the position and orientation of this camera to match those of another camera.
   *
   * \param rhs The other camera.
   * \return    This camera, after it has been moved.
   */
  virtual MoveableCamera& set_from(const Camera& rhs) = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MoveableCamera> MoveableCamera_Ptr;
typedef boost::shared_ptr<const MoveableCamera> MoveableCamera_CPtr;

}

#endif
