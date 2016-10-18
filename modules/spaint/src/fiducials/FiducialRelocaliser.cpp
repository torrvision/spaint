/**
 * spaint: FiducialRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/FiducialRelocaliser.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::optional<ORUtils::SE3Pose> FiducialRelocaliser::estimate_pose(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                                     const std::map<std::string,FiducialMeasurement>& measurements)
{
  // Compute a set of camera pose hypotheses based on the correspondences between the live measurements and the known fiducials.
  std::map<std::string,ORUtils::SE3Pose> cameraPoseHypotheses;
  for(std::map<std::string,FiducialMeasurement>::const_iterator it = measurements.begin(), iend = measurements.end(); it != iend; ++it)
  {
    // Try to find a fiducial corresponding to the measurement. If there isn't one, continue.
    std::map<std::string,Fiducial_Ptr>::const_iterator jt = fiducials.find(it->first);
    if(jt == fiducials.end()) continue;

    // Use the eye pose from the measurement and the world pose from the fiducial to compute a camera pose hypothesis.
    ORUtils::SE3Pose hypothesis;
    // TODO

    // Add the camera pose hypothesis to the set.
    cameraPoseHypotheses.insert(std::make_pair(it->first, hypothesis));
  }

  // Try to find a best camera pose hypothesis using exhaustive search (there aren't many hypotheses, so we don't need to use RANSAC).
  std::string bestHypothesis;
  std::map<std::string,ORUtils::SE3Pose> inliersForBestHypothesis;
  for(std::map<std::string,ORUtils::SE3Pose>::const_iterator it = cameraPoseHypotheses.begin(), iend = cameraPoseHypotheses.end(); it != iend; ++it)
  {
    // Calculate the inliers for the hypothesis.
    std::map<std::string,ORUtils::SE3Pose> inliers;
    for(std::map<std::string,ORUtils::SE3Pose>::const_iterator jt = cameraPoseHypotheses.begin(), jend = cameraPoseHypotheses.end(); jt != jend; ++jt)
    {
      // TODO
    }

    // Update the current best hypothesis as necessary.
    if(inliers.size() > inliersForBestHypothesis.size())
    {
      bestHypothesis = it->first;
      inliersForBestHypothesis = inliers;
    }
  }

  // If there isn't a best hypothesis, exit.
  if(bestHypothesis == "") return boost::none;

  // Otherwise, optimise the best hypothesis using its inliers and return it.
  // TODO
  return boost::none;
}

}
