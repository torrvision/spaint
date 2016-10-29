/**
 * spaint: FiducialRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/FiducialRelocaliser.h"

#include <vector>

#include <ITMLib/Utils/ITMMath.h>

#include "geometry/GeometryUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::optional<ORUtils::SE3Pose> FiducialRelocaliser::estimate_pose(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                                     const std::map<std::string,FiducialMeasurement>& measurements)
{
  // Compute a set of camera pose hypotheses based on the correspondences between the live measurements and the known fiducials.
  std::map<std::string,ORUtils::SE3Pose> cameraPoseHypotheses = compute_hypotheses(fiducials, measurements);

  // Try to find a best camera pose hypothesis using exhaustive search.
  std::map<std::string,ORUtils::SE3Pose> inliersForBestHypothesis;
  std::string bestHypothesis = find_best_hypothesis(cameraPoseHypotheses, inliersForBestHypothesis);

  // If a best hypothesis was found, refine it using its inliers and return it; if not, return none.
  if(bestHypothesis != "") return refine_best_hypothesis(inliersForBestHypothesis);
  else return boost::none;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

std::map<std::string,ORUtils::SE3Pose> FiducialRelocaliser::compute_hypotheses(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                                               const std::map<std::string,FiducialMeasurement>& measurements)
{
  std::map<std::string,ORUtils::SE3Pose> cameraPoseHypotheses;

  // For each measurement:
  for(std::map<std::string,FiducialMeasurement>::const_iterator it = measurements.begin(), iend = measurements.end(); it != iend; ++it)
  {
    // Try to find a stable fiducial corresponding to the measurement. If there isn't one, continue.
    std::map<std::string,Fiducial_Ptr>::const_iterator jt = fiducials.find(it->first);
    if(jt == fiducials.end() || jt->second->confidence() < Fiducial::stable_confidence()) continue;

    // Try to get the pose of the measurement in eye space. If it isn't available, continue.
    boost::optional<ORUtils::SE3Pose> fiducialPoseEye = it->second.pose_eye();
    if(!fiducialPoseEye) continue;

    // Use the eye pose from the measurement and the world pose from the fiducial to compute a camera pose hypothesis.
    ORUtils::SE3Pose fiducialPoseWorld = jt->second->pose();
    ORUtils::SE3Pose cameraPoseHypothesis(fiducialPoseEye->GetInvM() * fiducialPoseWorld.GetM());

    // Add the camera pose hypothesis to the set.
    cameraPoseHypotheses.insert(std::make_pair(it->first, cameraPoseHypothesis));
  }

  return cameraPoseHypotheses;
}

std::string FiducialRelocaliser::find_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& cameraPoseHypotheses,
                                                      std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis)
{
  std::string bestHypothesis;

  // For each camera pose hypothesis:
  for(std::map<std::string,ORUtils::SE3Pose>::const_iterator it = cameraPoseHypotheses.begin(), iend = cameraPoseHypotheses.end(); it != iend; ++it)
  {
    // Calculate the inliers for the hypothesis.
    std::map<std::string,ORUtils::SE3Pose> inliers;
    for(std::map<std::string,ORUtils::SE3Pose>::const_iterator jt = cameraPoseHypotheses.begin(), jend = cameraPoseHypotheses.end(); jt != jend; ++jt)
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

ORUtils::SE3Pose FiducialRelocaliser::refine_best_hypothesis(const std::map<std::string,ORUtils::SE3Pose>& inliersForBestHypothesis)
{
  std::vector<DualQuatd> dqs;
  std::vector<double> weights;
  const int count = static_cast<int>(inliersForBestHypothesis.size());

  // Compute a uniformly-weighted linear blend of all of the inlier poses and return it.
  const double weight = 1.0 / count;
  for(std::map<std::string,ORUtils::SE3Pose>::const_iterator it = inliersForBestHypothesis.begin(), iend = inliersForBestHypothesis.end(); it != iend; ++it)
  {
    dqs.push_back(GeometryUtil::pose_to_dual_quat<double>(it->second));
    weights.push_back(weight);
  }

  return GeometryUtil::dual_quat_to_pose(DualQuatd::linear_blend(&dqs[0], &weights[0], count));
}

}
