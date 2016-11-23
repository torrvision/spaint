/**
 * spaint: ExhaustivePoseRefiner.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_EXHAUSTIVEPOSEREFINER
#define H_SPAINT_EXHAUSTIVEPOSEREFINER

#include <map>

#include <ORUtils/SE3Pose.h>

namespace spaint {

/**
 * \brief This struct can be used to refine a set of (potentially diverse) pose hypotheses into a single pose.
 */
struct ExhaustivePoseRefiner
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Blends a set of poses together to construct a refined pose.
   *
   * \param poses The poses to blend.
   * \return      The refined pose.
   */
  static ORUtils::SE3Pose blend_poses(const std::map<std::string,ORUtils::SE3Pose>& poses);

  /**
   * \brief Finds a pose hypothesis with the greatest number of inliers.
   *
   * \param poseHypotheses            The set of pose hypotheses from which to choose the best hypothesis.
   * \param inliersForBestHypothesis  A place in which to store the inliers for the best hypothesis.
   * \return                          The ID of the best hypothesis.
   */
  static std::string find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& poseHypotheses,
                                          std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis);
};

}

#endif
