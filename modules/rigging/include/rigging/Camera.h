/**
 * rigging: Camera.h
 */

#ifndef H_RIGGING_CAMERA
#define H_RIGGING_CAMERA

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace rigging {

/**
 * \brief An instance of a class deriving from this one represents a camera in 3D space.
 *
 * Cameras are defined with a position and three mutually-orthogonal axes,
 * namely n (points in the direction faced by the camera), u (points to
 * the left of the camera) and v (points to the top of the camera).
 */
class Camera
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the camera.
   */
  virtual ~Camera() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a vector pointing in the direction faced by the camera.
   *
   * \return  A vector pointing in the direction faced by the camera.
   */
  virtual Eigen::Vector3f n() const = 0;

  /**
   * \brief Gets the position of the camera.
   *
   * \return  The position of the camera.
   */
  virtual Eigen::Vector3f p() const = 0;

  /**
   * \brief Gets a vector pointing to the left of the camera.
   *
   * \return  A vector pointing to the left of the camera.
   */
  virtual Eigen::Vector3f u() const = 0;

  /**
   * \brief Gets a vector pointing to the top of the camera.
   *
   * \return  A vector pointing to the top of the camera.
   */
  virtual Eigen::Vector3f v() const = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Camera> Camera_Ptr;
typedef boost::shared_ptr<const Camera> Camera_CPtr;

}

#endif
