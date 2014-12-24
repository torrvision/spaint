/**
 * spaint: CompositeCamera.h
 */

#ifndef H_SPAINT_COMPOSITECAMERA
#define H_SPAINT_COMPOSITECAMERA

#include <map>
#include <string>

#include "Camera.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to represent a "composite" camera (i.e. a camera rig) that contains several other cameras.
 *
 * A composite camera consists of a single primary camera that controls the position and orientation of the composite,
 * and a number of secondary cameras whose positions and orientations are based on that camera.
 */
class CompositeCamera : public Camera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The primary camera, which controls the position and orientation of the composite. */
  Camera_Ptr m_primaryCamera;

  /** The transformations for the secondary cameras, as a map from camera names -> transformations. */
  std::map<std::string,Eigen::Matrix4f> m_secondaryCameraTransformations;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a composite camera.
   *
   * \param primaryCamera The primary camera, which controls the position and orientation of the composite.
   */
  explicit CompositeCamera(const Camera_Ptr& primaryCamera);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a secondary camera to the composite.
   *
   * \param name            The name to give the secondary camera.
   * \param transformation  The transformation mapping points in the secondary camera's coordinate frame into that of the primary camera.
   */
  void add_secondary_camera(const std::string& name, const Eigen::Matrix4f& transformation);

  /**
   * \brief Gets a simple camera corresponding to the primary camera in the composite.
   *
   * \return  A simple camera corresponding to the primary camera in the composite.
   */
  SimpleCamera get_primary_camera() const;

  /**
   * \brief Gets a simple camera corresponding to the specified secondary camera in the composite.
   *
   * \param name  The name of the secondary camera.
   * \return      A simple camera corresponding to the specified secondary camera.
   * \throws      TODO
   */
  SimpleCamera get_secondary_camera(const std::string& name) const;

  /**
   * \brief Removes the secondary camera with the specified name from the composite.
   *
   * \param name  The name of the secondary camera.
   * \throws      TODO
   */
  void remove_secondary_camera(const std::string& name);
};

}

#endif
