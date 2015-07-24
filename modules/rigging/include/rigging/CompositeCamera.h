/**
 * rigging: CompositeCamera.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RIGGING_COMPOSITECAMERA
#define H_RIGGING_COMPOSITECAMERA

#include <map>
#include <string>

#include "SimpleCamera.h"

namespace rigging {

/**
 * \brief An instance of this class can be used to represent a "composite" camera (i.e. a camera rig) consisting of several other cameras.
 *
 * A composite camera consists of a single primary camera that controls the position and orientation of the composite,
 * and a number of secondary cameras that are generally directly or indirectly based on that camera.
 */
class CompositeCamera : public MoveableCamera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The primary camera, which controls the position and orientation of the composite. */
  SimpleCamera m_primaryCamera;

  /** The secondary cameras. */
  std::map<std::string,Camera_CPtr> m_secondaryCameras;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a composite camera.
   *
   * \param position  The position of the camera.
   * \param look      A vector pointing in the direction faced by the camera.
   * \param up        The "up" direction for the camera.
   */
  CompositeCamera(const Eigen::Vector3f& position, const Eigen::Vector3f& look, const Eigen::Vector3f& up);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a secondary camera to the composite.
   *
   * \param name                The name to give the secondary camera.
   * \param camera              The secondary camera.
   * \throws std::runtime_error If the composite already contains a camera with the specified name.
   */
  void add_secondary_camera(const std::string& name, const Camera_CPtr& camera);

  /**
   * \brief Gets the secondary camera with the specified name.
   *
   * \param name                The name of the secondary camera to get.
   * \return                    The secondary camera with the specified name, if it exists.
   * \throws std::runtime_error If the composite does not contain a camera with the specified name.
   */
  const Camera_CPtr& get_secondary_camera(const std::string& name) const;

  /** Override */
  CompositeCamera& move(const Eigen::Vector3f& dir, float delta);

  /** Override */
  CompositeCamera& move_n(float delta);

  /** Override */
  CompositeCamera& move_u(float delta);

  /** Override */
  CompositeCamera& move_v(float delta);

  /** Override */
  Eigen::Vector3f n() const;

  /** Override */
  Eigen::Vector3f p() const;

  /**
   * \brief Removes the secondary camera with the specified name from the composite.
   *
   * \param name                The name of the camera to remove.
   * \throws std::runtime_error If the composite does not contain a camera with the specified name.
   */
  void remove_secondary_camera(const std::string& name);

  /** Override */
  CompositeCamera& rotate(const Eigen::Vector3f& axis, float angle);

  /** Override */
  CompositeCamera& set_from(const Camera& rhs);

  /** Override */
  Eigen::Vector3f u() const;

  /** Override */
  Eigen::Vector3f v() const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CompositeCamera> CompositeCamera_Ptr;
typedef boost::shared_ptr<const CompositeCamera> CompositeCamera_CPtr;

}

#endif
