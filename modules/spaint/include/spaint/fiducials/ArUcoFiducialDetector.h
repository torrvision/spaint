/**
 * spaint: ArUcoFiducialDetector.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_ARUCOFIDUCIALDETECTOR
#define H_SPAINT_ARUCOFIDUCIALDETECTOR

#include <boost/optional.hpp>

#include <opencv2/opencv.hpp>

#include "FiducialDetector.h"
#include "../picking/interface/Picker.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to detect ArUco fiducials in a 3D scene.
 */
class ArUcoFiducialDetector : public FiducialDetector
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The picker used when estimating poses from the scene raycast. */
  mutable Picker_CPtr m_picker;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::map<std::string,FiducialMeasurement> detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& pose, const VoxelRenderState_CPtr& renderState,
                                                                     PoseEstimationMode poseEstimationMode) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  std::vector<boost::optional<FiducialMeasurement> > construct_measurements_from_colour(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                                                        const View_CPtr& view, const ORUtils::SE3Pose& pose) const;

  /**
   * \brief TODO
   */
  std::vector<boost::optional<FiducialMeasurement> > construct_measurements_from_depth(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                                                       const View_CPtr& view, const ORUtils::SE3Pose& pose) const;

  /**
   * \brief TODO
   */
  std::vector<boost::optional<FiducialMeasurement> > construct_measurements_from_raycast(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                                                         const VoxelRenderState_CPtr& renderState) const;

  /**
   * \brief TODO
   */
  boost::optional<Vector3f> pick_corner(const cv::Point2f& corner, const VoxelRenderState_CPtr& renderState) const;
};

}

#endif
