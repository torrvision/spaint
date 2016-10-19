/**
 * spaint: GeometryUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GEOMETRYUTIL
#define H_SPAINT_GEOMETRYUTIL

#include <cmath>

#include <ORUtils/SE3Pose.h>

#include "DualQuaternion.h"

namespace spaint {

/**
 * \brief This struct provides a number of useful geometric utility functions.
 */
struct GeometryUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

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
   * \brief Converts an SE(3) pose to a dual quaternion.
   *
   * \param pose  The SE(3) pose.
   * \return      The corresponding dual quaternion.
   */
  template <typename T>
  static DualQuaternion<T> pose_to_dual_quat(const ORUtils::SE3Pose& pose)
  {
    Vector3f r, t;
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
