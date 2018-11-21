/**
 * spaint: ViconFiducialDetector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_SPAINT_VICONFIDUCIALDETECTOR
#define H_SPAINT_VICONFIDUCIALDETECTOR

#include <itmx/util/ViconInterface.h>

#include "FiducialDetector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to detect Vicon fiducials (based on multiple Vicon markers) in a 3D scene.
 */
class ViconFiducialDetector : public FiducialDetector
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The fiducial detector used to determine the transformation between Vicon space and world space. */
  FiducialDetector_CPtr m_calibratingDetector;

  /** The Vicon interface. */
  itmx::ViconInterface_CPtr m_vicon;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Vicon fiducial detector.
   *
   * \param vicon               The Vicon interface.
   * \param calibratingDetector The fiducial detector used to determine the transformation between Vicon space and world space.
   */
  explicit ViconFiducialDetector(const itmx::ViconInterface_CPtr& vicon, const FiducialDetector_CPtr& calibratingDetector);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::map<std::string,FiducialMeasurement> detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& depthPose) const;
};

}

#endif
