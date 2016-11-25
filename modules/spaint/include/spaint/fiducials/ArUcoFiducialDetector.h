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

  /** The settings to use for InfiniTAM. */
  Settings_CPtr m_settings;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an ArUco fiducial detector.
   *
   * \param settings  The settings to use for InfiniTAM.
   */
  explicit ArUcoFiducialDetector(const Settings_CPtr& settings);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::map<std::string,FiducialMeasurement> detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& pose, const VoxelRenderState_CPtr& renderState,
                                                                     PoseEstimationMode poseEstimationMode) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Constructs a set of fiducial measurements by directly estimating poses for the fiducials from the live colour image.
   *
   * \note  The poses are estimated using the OpenCV ArUco library, and are much less accurate than the poses we can get using
   *        either the live depth image or a raycast of the scene. The sole advantage of this approach is that it can produce
   *        poses when neither of those two sources of information are available.
   *
   * \param ids     The IDs of the fiducials that have been detected in the live colour image.
   * \param corners The corners of the fiducials that have been detected in the live colour image.
   * \param view    The view of the scene containing the live images.
   * \param pose    The current estimate of the camera pose (used to map between eye space and world space).
   * \return        The constructed set of fiducial measurements.
   */
  std::vector<boost::optional<FiducialMeasurement> > construct_measurements_from_colour(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                                                        const View_CPtr& view, const ORUtils::SE3Pose& pose) const;

  /**
   * \brief Constructs a set of fiducial measurements by back-projecting the detected fiducial corners in the live colour image
   *        into 3D using depth values from the live depth image, and then using the back-projected corners to determine poses
   *        for the fiducials in both eye and world space.
   *
   * \param ids     The IDs of the fiducials that have been detected in the live colour image.
   * \param corners The corners of the fiducials that have been detected in the live colour image.
   * \param view    The view of the scene containing the live images.
   * \param pose    The current estimate of the camera pose (used to map between eye space and world space).
   * \return        The constructed set of fiducial measurements.
   */
  std::vector<boost::optional<FiducialMeasurement> > construct_measurements_from_depth(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                                                       const View_CPtr& view, const ORUtils::SE3Pose& pose) const;

  /**
   * \brief Constructs a set of fiducial measurements by looking up in a raycast of the scene the 3D points in world space
   *        that correspond to the detected fiducial corners in the live colour image, and then using these 3D points to
   *        determine poses for the fiducials in both world and eye space.
   *
   * \param ids         The IDs of the fiducials that have been detected in the live colour image.
   * \param corners     The corners of the fiducials that have been detected in the live colour image.
   * \param renderState The render state containing the scene raycast.
   * \param pose        The current estimate of the camera pose (used to map between world space and eye space).
   * \return            The constructed set of fiducial measurements.
   */
  std::vector<boost::optional<FiducialMeasurement> > construct_measurements_from_raycast(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                                                         const VoxelRenderState_CPtr& renderState, const ORUtils::SE3Pose& pose) const;

  /**
   * \brief Tries to determine the 3D point in eye space that corresponds to a fiducial corner in the live colour image
   *        by back-projecting into 3D using the depth value from the live depth image.
   *
   * \param corner  The fiducial corner in the live colour image.
   * \param view    The view of the scene containing the live images.
   * \return        The 3D point in eye space corresponding to the fiducial corner (if any), or boost::none otherwise.
   */
  boost::optional<Vector3f> pick_corner_from_depth(const cv::Point2f& corner, const View_CPtr& view) const;

  /**
   * \brief Tries to determine the 3D point in world space that corresponds to a fiducial corner in the live colour image
   *        by looking it up in a raycast of the scene from the pose of the depth camera.
   *
   * \param corner      The fiducial corner in the live colour image.
   * \param renderState The render state containing the scene raycast.
   * \return            The 3D point in world space corresponding to the fiducial corner (if any), or boost::none otherwise.
   */
  boost::optional<Vector3f> pick_corner_from_raycast(const cv::Point2f& corner, const VoxelRenderState_CPtr& renderState) const;

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Attempts to make a pose matrix from three corner points of an ArUco marker.
   *
   * \param v0  The first corner point.
   * \param v1  The second corner point.
   * \param v2  The third corner point.
   * \return    The pose matrix, if all three corner points exist, or boost::none otherwise.
   */
  static boost::optional<ORUtils::SE3Pose> make_pose_from_corners(const boost::optional<Vector3f>& v0,
                                                                  const boost::optional<Vector3f>& v1,
                                                                  const boost::optional<Vector3f>& v2);
};

}

#endif
