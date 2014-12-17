/**
 * spaint: Camera.h
 */

#ifndef H_SPAINT_CAMERA
#define H_SPAINT_CAMERA

#include <Eigen/Dense>

namespace spaint {

/**
 * \brief An instance of this class represents a camera for a 3D view.
 *
 * Cameras are defined with a position and three mutually-orthogonal axes,
 * namely n (points in the direction faced by the camera), u (points to
 * the left of the camera) and v (points to the top of the camera).
 */
class Camera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A vector pointing in the direction faced by the camera. */
  Eigen::Vector3f m_n;

  /** The position of the camera. */
  Eigen::Vector3f m_position;

  /** A vector pointing to the left of the camera. */
  Eigen::Vector3f m_u;

  /** A vector pointing to the top of the camera. */
  Eigen::Vector3f m_v;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a new camera.
   *
   * \param position  The position of the camera.
   * \param look      A vector pointing in the direction faced by the camera.
   * \param up        The "up" direction for the camera.
   */
  Camera(const Eigen::Vector3f& position, const Eigen::Vector3f& look, const Eigen::Vector3f& up);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Moves the camera by the specified displacement in the n direction.
   *
   * \param delta The displacement by which to move.
   */
  Camera& move_n(float delta);

  /**
   * \brief Moves the camera by the specified displacement in the u direction.
   *
   * \param delta The displacement by which to move.
   */
  Camera& move_u(float delta);

  /**
   * \brief Moves the camera by the specified displacement in the v direction.
   *
   * \param delta The displacement by which to move.
   */
  Camera& move_v(float delta);

  /**
   * \brief Rotates the camera anticlockwise by the specified angle about the specified axis.
   *
   * \param axis  The axis about which to rotate.
   * \param angle The angle by which to rotate (in radians).
   */
  Camera& rotate(const Eigen::Vector3f& axis, float angle);
};

}

#endif
