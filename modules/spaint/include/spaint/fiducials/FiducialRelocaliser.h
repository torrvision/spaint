/**
 * spaint: FiducialRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_FIDUCIALRELOCALISER
#define H_SPAINT_FIDUCIALRELOCALISER

#include <map>

#include "Fiducial.h"

namespace spaint {

/**
 * \brief This class can be used to relocalise a camera in a scene using fiducial markers.
 */
class FiducialRelocaliser
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Tries to estimate the current camera pose by using correspondences between known fiducials in the scene
   *        and fiducial measurements in the live frame.
   *
   * \param fiducials     The known fiducials in the scene.
   * \param measurements  The fiducial measurements in the live frame.
   * \return              An estimate of the camera pose, if possible, or boost::none otherwise.
   */
  static boost::optional<ORUtils::SE3Pose> estimate_pose(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                         const std::map<std::string,FiducialMeasurement>& measurements);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  static std::map<std::string,ORUtils::SE3Pose> compute_hypotheses(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                                   const std::map<std::string,FiducialMeasurement>& measurements);

  /**
   * \brief TODO
   */
  static std::string find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& cameraPoseHypotheses,
                                          std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis);

  /**
   * \brief TODO
   */
  static ORUtils::SE3Pose refine_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis);
};

}

#endif
