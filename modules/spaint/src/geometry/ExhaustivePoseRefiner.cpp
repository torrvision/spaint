/**
 * spaint: ExhaustivePoseRefiner.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "geometry/ExhaustivePoseRefiner.h"

#include <vector>

#include "geometry/GeometryUtil.h"

namespace spaint {

ORUtils::SE3Pose ExhaustivePoseRefiner::blend_poses(const std::map<std::string,ORUtils::SE3Pose>& poses)
{
  std::vector<DualQuatd> dqs;
  std::vector<double> weights;
  const int count = static_cast<int>(poses.size());

  // Compute a uniformly-weighted linear blend of all of the poses and return it.
  const double weight = 1.0 / count;
  for(std::map<std::string,ORUtils::SE3Pose>::const_iterator it = poses.begin(), iend = poses.end(); it != iend; ++it)
  {
    dqs.push_back(GeometryUtil::pose_to_dual_quat<double>(it->second));
    weights.push_back(weight);
  }

  return GeometryUtil::dual_quat_to_pose(DualQuatd::linear_blend(&dqs[0], &weights[0], count));
}

std::string ExhaustivePoseRefiner::find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& poseHypotheses,
                                                        std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis)
{
  std::string bestHypothesis;

  // For each pose hypothesis:
  for(std::map<std::string,ORUtils::SE3Pose>::const_iterator it = poseHypotheses.begin(), iend = poseHypotheses.end(); it != iend; ++it)
  {
    // Calculate the inliers for the hypothesis.
    std::map<std::string,ORUtils::SE3Pose> inliers;
    for(std::map<std::string,ORUtils::SE3Pose>::const_iterator jt = poseHypotheses.begin(), jend = poseHypotheses.end(); jt != jend; ++jt)
    {
      if(GeometryUtil::poses_are_similar(it->second, jt->second))
      {
        inliers.insert(*jt);
      }
    }

    // Update the current best hypothesis as necessary.
    if(inliers.size() > inliersForBestHypothesis.size())
    {
      bestHypothesis = it->first;
      inliersForBestHypothesis = inliers;
    }
  }

  return bestHypothesis;
}

}
