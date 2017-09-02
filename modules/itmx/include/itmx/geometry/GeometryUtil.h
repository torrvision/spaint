/**
 * itmx: GeometryUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_ITMX_GEOMETRYUTIL
#define H_ITMX_GEOMETRYUTIL

#include <cmath>
#include <map>
#include <vector>

#include <ORUtils/SE3Pose.h>

#include "DualQuaternion.h"

namespace itmx {

/**
 * \brief This struct provides a number of useful geometric utility functions.
 */
struct GeometryUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Linearly blends a set of poses together to construct a refined pose.
   *
   * \param poses The poses to blend.
   * \return      The refined pose.
   */
  static ORUtils::SE3Pose blend_poses(const std::vector<ORUtils::SE3Pose>& poses);

  /**
   * \brief Converts a dual quaternion to an SE(3) pose.
   *
   * \param dq  The dual quaternion.
   * \return    The corresponding SE(3) pose.
   */
  template <typename T>
  static ORUtils::SE3Pose dual_quat_to_pose(const DualQuaternion<T>& dq)
  {
    ORUtils::SE3Pose pose;
    pose.SetFrom(dq.get_translation().toFloat(), dq.get_rotation().toFloat());
    return pose;
  }

  /**
   * \brief Finds a pose hypothesis with the greatest number of inliers from a set of such hypotheses.
   *
   * \param poseHypotheses            The set of pose hypotheses from which to choose the best hypothesis.
   * \param inliersForBestHypothesis  A place in which to store the inliers for the best hypothesis.
   * \param rotThreshold              The angular threshold to use when comparing rotations.
   * \param transThreshold            The distance threshold to use when comparing translations.
   * \return                          The best hypothesis.
   */
  static ORUtils::SE3Pose find_best_hypothesis(const std::vector<ORUtils::SE3Pose>& poseHypotheses,
                                               std::vector<ORUtils::SE3Pose>& inliersForBestHypothesis,
                                               double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);

  /**
   * \brief Finds a pose hypothesis with the greatest number of inliers from a set of such hypotheses.
   *
   * \param poseHypotheses            The set of pose hypotheses from which to choose the best hypothesis.
   * \param inliersForBestHypothesis  A place in which to store the inliers for the best hypothesis.
   * \param rotThreshold              The angular threshold to use when comparing rotations.
   * \param transThreshold            The distance threshold to use when comparing translations.
   * \return                          The ID of the best hypothesis.
   */
  static std::string find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& poseHypotheses,
                                          std::vector<ORUtils::SE3Pose>& inliersForBestHypothesis,
                                          double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);

  /**
   * \brief Converts an SE(3) pose to a dual quaternion.
   *
   * \param pose  The SE(3) pose.
   * \return      The corresponding dual quaternion.
   */
  template <typename T>
  static DualQuaternion<T> pose_to_dual_quat(const ORUtils::SE3Pose& pose)
  {
    ORUtils::Vector3<float> r, t;
    pose.GetParams(t, r);

    ORUtils::Vector3<T> typedR(static_cast<T>(r.x), static_cast<T>(r.y), static_cast<T>(r.z));
    ORUtils::Vector3<T> typedT(static_cast<T>(t.x), static_cast<T>(t.y), static_cast<T>(t.z));

    return DualQuaternion<T>::from_translation(typedT) * DualQuaternion<T>::from_rotation(typedR);
  }

  /**
   * \brief Determines whether or not two SE(3) poses are sufficiently similar.
   *
   * Similarity is defined in terms of both the rotations and translations involved. Rotation similarity is
   * assessed by looking at the relative rotation mapping one of the two input rotations to the other, and
   * thresholding the angle involved. Translation similarity is assessed by thresholding the distance between
   * the two input translations. Iff both their rotations and translations are similar, so are the poses.
   *
   * \param pose1           The first pose.
   * \param pose2           The second pose.
   * \param rotThreshold    The angular threshold to use when comparing the rotations.
   * \param transThreshold  The distance threshold to use when comparing the translations.
   * \return                true, if the poses are sufficiently similar, or false otherwise.
   */
  static bool poses_are_similar(const ORUtils::SE3Pose& pose1, const ORUtils::SE3Pose& pose2, double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);
};

}

#endif
