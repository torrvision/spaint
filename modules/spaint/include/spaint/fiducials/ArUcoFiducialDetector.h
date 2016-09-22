/**
 * spaint: ArUcoFiducialDetector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_ARUCOFIDUCIALDETECTOR
#define H_SPAINT_ARUCOFIDUCIALDETECTOR

#include <opencv2/opencv.hpp>

#include "FiducialDetector.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to detect ArUco fiducials in a 3D scene.
 */
class ArUcoFiducialDetector : public FiducialDetector
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::map<std::string,Fiducial> detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& pose) const;

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  static std::vector<ORUtils::SE3Pose> estimate_poses_from_tsdf(const std::vector<std::vector<cv::Point2f> >& corners, const View_CPtr& view,
                                                                const ORUtils::SE3Pose& pose, const VoxelRenderState_CPtr& renderState);

  /**
   * \brief TODO
   */
  static std::vector<ORUtils::SE3Pose> estimate_poses_from_view(const std::vector<std::vector<cv::Point2f> >& corners, const View_CPtr& view,
                                                                const ORUtils::SE3Pose& pose);
};

}

#endif
