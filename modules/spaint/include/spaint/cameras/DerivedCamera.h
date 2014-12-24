/**
 * spaint: DerivedCamera.h
 */

#ifndef H_SPAINT_DERIVEDCAMERA
#define H_SPAINT_DERIVEDCAMERA

#include <Eigen/Dense>

#include "Camera.h"

namespace spaint {

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

  /** The transformation mapping points in this camera's coordinate frame into that of the base camera. */
  Eigen::Matrix4f m_transformation;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a derived camera.
   *
   * \param baseCamera      The camera on which this derived camera is based.
   * \param transformation  The transformation mapping points in this camera's coordinate frame into that of the base camera.
   */
  DerivedCamera(const Camera_CPtr& baseCamera, const Eigen::Matrix4f& transformation);

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
};

}

#endif
