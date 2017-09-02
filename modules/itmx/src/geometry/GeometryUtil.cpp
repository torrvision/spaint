/**
 * itmx: GeometryUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "geometry/GeometryUtil.h"

#include <ITMLib/Utils/ITMMath.h>

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

#include "geometry/DualQuaternion.h"

namespace itmx {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ORUtils::SE3Pose GeometryUtil::blend_poses(const std::vector<ORUtils::SE3Pose>& poses)
{
  std::vector<DualQuatd> dqs;
  std::vector<double> weights;
  const int count = static_cast<int>(poses.size());

  // Compute a uniformly-weighted linear blend of all of the poses and return it.
  const double weight = 1.0 / count;
  for(size_t i = 0; i < count; ++i)
  {
    dqs.push_back(pose_to_dual_quat<double>(poses[i]));
    weights.push_back(weight);
  }

  return dual_quat_to_pose(DualQuatd::linear_blend(&dqs[0], &weights[0], count));
}

ORUtils::SE3Pose GeometryUtil::find_best_hypothesis(const std::vector<ORUtils::SE3Pose>& poseHypotheses,
                                                    std::vector<ORUtils::SE3Pose>& inliersForBestHypothesis,
                                                    double rotThreshold, float transThreshold)
{
  std::map<std::string,ORUtils::SE3Pose> poseHypothesesMap;
  for(size_t i = 0, size = poseHypotheses.size(); i < size; ++i)
  {
    poseHypothesesMap.insert(std::make_pair(boost::lexical_cast<std::string>(i), poseHypotheses[i]));
  }

  std::string bestHypothesis = find_best_hypothesis(poseHypothesesMap, inliersForBestHypothesis, rotThreshold, transThreshold);
  return MapUtil::lookup(poseHypothesesMap, bestHypothesis);
}

std::string GeometryUtil::find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& poseHypotheses,
                                               std::vector<ORUtils::SE3Pose>& inliersForBestHypothesis,
                                               double rotThreshold, float transThreshold)
{
  std::string bestHypothesis;

  // For each pose hypothesis:
  for(std::map<std::string,ORUtils::SE3Pose>::const_iterator it = poseHypotheses.begin(), iend = poseHypotheses.end(); it != iend; ++it)
  {
    // Calculate the inliers for the hypothesis.
    std::vector<ORUtils::SE3Pose> inliers;
    for(std::map<std::string,ORUtils::SE3Pose>::const_iterator jt = poseHypotheses.begin(), jend = poseHypotheses.end(); jt != jend; ++jt)
    {
      if(poses_are_similar(it->second, jt->second, rotThreshold, transThreshold))
      {
        inliers.push_back(jt->second);
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

bool GeometryUtil::poses_are_similar(const ORUtils::SE3Pose& pose1, const ORUtils::SE3Pose& pose2, double rotThreshold, float transThreshold)
{
  Vector3f r1, t1, r2, t2;
  pose1.GetParams(t1, r1);
  pose2.GetParams(t2, r2);

  double rot = DualQuatd::angle_between_rotations(DualQuatd::from_rotation(r1), DualQuatd::from_rotation(r2));
  float trans = length(t1 - t2);

  return rot <= rotThreshold && trans <= transThreshold;
}

}
