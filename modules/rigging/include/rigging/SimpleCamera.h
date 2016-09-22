/**
 * rigging: SimpleCamera.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RIGGING_SIMPLECAMERA
#define H_RIGGING_SIMPLECAMERA

#include "MoveableCamera.h"

namespace rigging {

/**
 * \brief An instance of this class represents a simple, moveable camera in 3D space.
 */
class SimpleCamera : public MoveableCamera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A (normalised) vector pointing in the direction faced by the camera. */
  Eigen::Vector3f m_n;

  /** The position of the camera. */
  Eigen::Vector3f m_position;

  /** A (normalised) vector pointing to the left of the camera. */
  Eigen::Vector3f m_u;

  /** A (normalised) vector pointing to the top of the camera. */
  Eigen::Vector3f m_v;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a simple camera.
   *
   * \param position  The position of the camera.
   * \param look      A vector pointing in the direction faced by the camera.
   * \param up        The "up" direction for the camera.
   */
  SimpleCamera(const Eigen::Vector3f& position, const Eigen::Vector3f& look, const Eigen::Vector3f& up);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  SimpleCamera& move(const Eigen::Vector3f& dir, float delta);

  /** Override */
  SimpleCamera& move_n(float delta);

  /** Override */
  SimpleCamera& move_u(float delta);

  /** Override */
  SimpleCamera& move_v(float delta);

  /** Override */
  Eigen::Vector3f n() const;

  /** Override */
  Eigen::Vector3f p() const;

  /** Override */
  SimpleCamera& rotate(const Eigen::Vector3f& axis, float angle);

  /** Override */
  SimpleCamera& set_from(const Camera& rhs);

  /** Override */
  Eigen::Vector3f u() const;

  /** Override */
  Eigen::Vector3f v() const;
};

}

#endif
