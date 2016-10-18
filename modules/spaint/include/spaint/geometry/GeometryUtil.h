/**
 * spaint: GeometryUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GEOMETRYUTIL
#define H_SPAINT_GEOMETRYUTIL

#include <cmath>

#include <ORUtils/SE3Pose.h>

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
  static bool poses_are_similar(const ORUtils::SE3Pose& pose1, const ORUtils::SE3Pose& pose2, double rotThreshold = 20 * M_PI / 180, float transThreshold = 0.05f);
};

}

#endif
