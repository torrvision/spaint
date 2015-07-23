/**
 * rigging: DerivedCamera.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RIGGING_DERIVEDCAMERA
#define H_RIGGING_DERIVEDCAMERA

#include "Camera.h"

namespace rigging {

/**
 * \brief An instance of this class can be used to represent a camera whose position and orientation are based on those of another camera.
 *
 * Note 1: Derived cameras are not themselves moveable; rather, they move when the camera on which they are based moves.
 * Note 2: From an object-oriented programming perspective, derived cameras can be seen as camera "decorators".
 */
class DerivedCamera : public Camera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The camera on which this derived camera is based. */
  Camera_CPtr m_baseCamera;

  /** The *camera-space* (u-v-n) rotation from the base camera's axes to those of the derived camera. */
  Eigen::Matrix3f m_rot;

  /** The *camera-space* (u-v-n) translation from the base camera's axes to those of the derived camera. */
  Eigen::Vector3f m_trans;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a derived camera.
   *
   * \param baseCamera  The camera on which this derived camera is based.
   * \param rot         The *camera-space* (u-v-n) rotation from the base camera's axes to those of the derived camera.
   * \param trans       The *camera-space* (u-v-n) translation from the base camera's axes to those of the derived camera.
   */
  DerivedCamera(const Camera_CPtr& baseCamera, const Eigen::Matrix3f& rot, const Eigen::Vector3f& trans);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  Eigen::Vector3f n() const;

  /** Override */
  Eigen::Vector3f p() const;

  /** Override */
  Eigen::Vector3f u() const;

  /** Override */
  Eigen::Vector3f v() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes the world-space rotation corresponding to the camera-space rotation we're using.
   *
   * \return  The world-space rotation corresponding to the camera-space rotation we're using.
   */
  Eigen::Matrix3f make_world_space_rotation() const;
};

}

#endif
