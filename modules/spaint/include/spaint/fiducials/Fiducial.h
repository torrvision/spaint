/**
 * spaint: Fiducial.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FIDUCIAL
#define H_SPAINT_FIDUCIAL

#include <boost/shared_ptr.hpp>

#include <ORUtils/SE3Pose.h>

namespace spaint {

/**
 * \brief An instance of this class represents a fiducial (a reference marker in a 3D scene).
 */
class Fiducial
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The ID of the fiducial. */
  std::string m_id;

  /** The pose of the fiducial in the 3D scene. */
  ORUtils::SE3Pose m_pose;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a fiducial.
   *
   * \param id    The ID of the fiducial.
   * \param pose  The pose of the fiducial in the 3D scene.
   */
  Fiducial(const std::string& id, const ORUtils::SE3Pose& pose);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the ID of the fiducial.
   *
   * \return  The ID of the fiducial.
   */
  const std::string& id() const;

  /**
   * \brief Gets the pose of the fiducial in the 3D scene.
   *
   * \return  The pose of the fiducial in the 3D scene.
   */
  const ORUtils::SE3Pose& pose() const;

  /**
   * \brief Updates the fiducial based on information from a new measurement.
   *
   * \param newFiducial         The fiducial containing the new measurement.
   * \throws std::runtime_error If the two fiducials do not have the same ID.
   */
  void update(const Fiducial& newFiducial);
};

}

#endif
