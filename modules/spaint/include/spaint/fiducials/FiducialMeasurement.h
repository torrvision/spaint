/**
 * spaint: FiducialMeasurement.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FIDUCIALMEASUREMENT
#define H_SPAINT_FIDUCIALMEASUREMENT

#include <boost/optional.hpp>

#include <ORUtils/SE3Pose.h>

namespace spaint {

/**
 * \brief An instance of this class represents a measurement of a fiducial (a reference marker in a 3D scene).
 */
class FiducialMeasurement
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The ID of the fiducial. */
  std::string m_id;

  /** The pose of the fiducial in eye space (if known). */
  boost::optional<ORUtils::SE3Pose> m_poseEye;

  /** The pose of the fiducial in world space (if known). */
  boost::optional<ORUtils::SE3Pose> m_poseWorld;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a fiducial measurement.
   *
   * \param id        The ID of the fiducial.
   * \param poseEye   The pose of the fiducial in eye space (if known).
   * \param poseWorld The pose of the fiducial in world space (if known).
   */
  FiducialMeasurement(const std::string& id, const boost::optional<ORUtils::SE3Pose>& poseEye, const boost::optional<ORUtils::SE3Pose>& poseWorld);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the ID of the fiducial.
   *
   * \return  The ID of the fiducial.
   */
  const std::string& id() const;

  /**
   * \brief Gets the pose of the fiducial in eye space (if known).
   *
   * \return  The pose of the fiducial in eye space (if known).
   */
  const boost::optional<ORUtils::SE3Pose>& pose_eye() const;

  /**
   * \brief Gets the pose of the fiducial in world space (if known).
   *
   * \return  The pose of the fiducial in world space (if known).
   */
  const boost::optional<ORUtils::SE3Pose>& pose_world() const;
};

}

#endif
