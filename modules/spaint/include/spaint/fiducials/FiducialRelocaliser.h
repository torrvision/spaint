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
   * \brief Finds correspondences between known fiducials in the scene and fiducial measurements in the live frame,
   *        and uses them to generate a set of camera pose hypotheses (at most one per fiducial).
   *
   * \param fiducials     The known fiducials in the scene.
   * \param measurements  The fiducial measurements in the live frame.
   * \return              A set of camera pose hypotheses, indexed by the IDs of the fiducials to which they correspond.
   */
  static std::map<std::string,ORUtils::SE3Pose> compute_hypotheses(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                                   const std::map<std::string,FiducialMeasurement>& measurements);

  /**
   * \brief Finds the camera pose hypothesis with the greatest number of inliers.
   *
   * \param cameraPoseHypotheses      The set of camera pose hypotheses from which to choose the best hypothesis.
   * \param inliersForBestHypothesis  A place in which to store the inliers for the best hypothesis.
   * \return                          The ID of the fiducial corresponding to the best hypothesis.
   */
  static std::string find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& cameraPoseHypotheses,
                                          std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis);

  /**
   * \brief Refines the best camera pose hypothesis by blending together all of its inliers.
   *
   * \param inliersForBestHypothesis  The inliers for the best hypothesis.
   * \return                          The refined best hypothesis.
   */
  static ORUtils::SE3Pose refine_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis);
};

}

#endif
