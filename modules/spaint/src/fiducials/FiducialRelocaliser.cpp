/**
 * spaint: FiducialRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/FiducialRelocaliser.h"

#include "geometry/ExhaustivePoseRefiner.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::optional<ORUtils::SE3Pose> FiducialRelocaliser::estimate_pose(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                                     const std::map<std::string,FiducialMeasurement>& measurements)
{
  // Compute a set of camera pose hypotheses based on the correspondences between the live measurements and the known fiducials.
  std::map<std::string,ORUtils::SE3Pose> cameraPoseHypotheses = compute_hypotheses(fiducials, measurements);

  // Try to find a best camera pose hypothesis using exhaustive search.
  std::map<std::string,ORUtils::SE3Pose> inliersForBestHypothesis;
  std::string bestHypothesis = ExhaustivePoseRefiner::find_best_hypothesis(cameraPoseHypotheses, inliersForBestHypothesis);

  // If a best hypothesis was found, return the result of blending its inliers together; if not, return none.
  if(bestHypothesis != "") return ExhaustivePoseRefiner::blend_poses(inliersForBestHypothesis);
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

}
