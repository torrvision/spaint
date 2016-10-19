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
 * \brief TODO
 */
struct GeometryUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief TODO
   */
  template <typename T>
  static ORUtils::SE3Pose dual_quat_to_pose(const DualQuaternion<T>& dq)
  {
    ORUtils::SE3Pose pose;
    pose.SetFrom(dq.get_translation().toFloat(), dq.get_rotation().toFloat());
    return pose;
  }

  /**
   * \brief TODO
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
   * \brief TODO
   */
  static bool poses_are_similar(const ORUtils::SE3Pose& pose1, const ORUtils::SE3Pose& pose2, double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);
};

}

#endif
