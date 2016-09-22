/**
 * spaint: ArUcoFiducialDetector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/ArUcoFiducialDetector.h"

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <ITMLib/Objects/Camera/ITMIntrinsics.h>
using namespace ITMLib;

#include "ocv/OpenCVUtil.h"
#include "picking/PickerFactory.h"
#include "util/CameraPoseConverter.h"
#include "util/MemoryBlockFactory.h"
using namespace rigging;

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::map<std::string,Fiducial> ArUcoFiducialDetector::detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& pose,
                                                                       const VoxelRenderState_CPtr& renderState) const
{
  std::map<std::string,Fiducial> fiducials;

  // Convert the current colour input image to OpenCV format.
  const ITMUChar4Image *rgb = view->rgb;
  rgb->UpdateHostFromDevice();
  cv::Mat3b rgbImage = OpenCVUtil::make_rgb_image(rgb->GetData(MEMORYDEVICE_CPU), rgb->noDims.x, rgb->noDims.y);

  // Detect any ArUco fiducials that are visible.
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<std::vector<cv::Point2f> > corners;
  std::vector<int> ids;
  cv::aruco::detectMarkers(rgbImage, dictionary, corners, ids);

#if 0
  // Visualise the detected fiducials for debugging purposes.
  cv::Mat3b markerImage = rgbImage.clone();
  cv::aruco::drawDetectedMarkers(markerImage, corners, ids);
  cv::imshow("Detected Markers", markerImage);
#endif

  // Estimate the poses of the fiducials in world space.
  std::vector<ORUtils::SE3Pose> fiducialPoses = estimate_poses_from_raycast(corners, ids, view, pose, renderState);

  for(size_t i = 0, size = ids.size(); i < size; ++i)
  {
    std::string id = boost::lexical_cast<std::string>(ids[i]);
    fiducials.insert(std::make_pair(id, Fiducial(id, fiducialPoses[i])));
  }

  return fiducials;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

std::vector<ORUtils::SE3Pose> ArUcoFiducialDetector::estimate_poses_from_raycast(const std::vector<std::vector<cv::Point2f> >& corners,
                                                                                 const std::vector<int>& ids,
                                                                                 const View_CPtr& view, const ORUtils::SE3Pose& pose,
                                                                                 const VoxelRenderState_CPtr& renderState) const
{
  std::vector<ORUtils::SE3Pose> fiducialPoses;

  for(size_t i = 0, size = corners.size(); i < size; ++i)
  {
    boost::optional<Vector3f> v0 = pick_corner(corners[i][3], renderState);
    boost::optional<Vector3f> v1 = pick_corner(corners[i][2], renderState);
    boost::optional<Vector3f> v2 = pick_corner(corners[i][0], renderState);

    ORUtils::SE3Pose fiducialPose;

    if(v0 && v1 && v2)
    {
      Vector3f xp = (*v1 - *v0).normalised();
      Vector3f yp = (*v2 - *v0).normalised();
      Vector3f zp = ORUtils::cross(xp, yp);
      yp = ORUtils::cross(zp, xp);

      SimpleCamera cam(
        Eigen::Vector3f(v0->x, v0->y, v0->z),
        Eigen::Vector3f(zp.x, zp.y, zp.z),
        Eigen::Vector3f(-yp.x, -yp.y, -yp.z)
      );

      fiducialPose = CameraPoseConverter::camera_to_pose(cam);
    }

    fiducialPoses.push_back(fiducialPose);
  }

  return fiducialPoses;
}

std::vector<ORUtils::SE3Pose> ArUcoFiducialDetector::estimate_poses_from_view(const std::vector<std::vector<cv::Point2f> >& corners,
                                                                              const View_CPtr& view, const ORUtils::SE3Pose& pose) const
{
  std::vector<ORUtils::SE3Pose> fiducialPoses;

  // Estimate the poses of the fiducials in eye space.
  const ITMIntrinsics& intrinsics = view->calib.intrinsics_rgb;
  cv::Mat1f cameraMatrix = cv::Mat1f::zeros(3, 3);
  cameraMatrix(0, 0) = intrinsics.projectionParamsSimple.fx;
  cameraMatrix(1, 1) = intrinsics.projectionParamsSimple.fy;
  cameraMatrix(0, 2) = intrinsics.projectionParamsSimple.px;
  cameraMatrix(1, 2) = intrinsics.projectionParamsSimple.py;
  cameraMatrix(2, 2) = 1.0f;

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, 0.02f, cameraMatrix, cv::noArray(), rvecs, tvecs);

  // Convert the poses of the fiducials into world space and return them.
  for(size_t i = 0, size = corners.size(); i < size; ++i)
  {
    cv::Mat1d rot;
    cv::Rodrigues(rvecs[i], rot);

    Matrix4f fiducialToEye(0.0f);
    for(int y = 0; y < 3; ++y)
    {
      for(int x = 0; x < 3; ++x)
      {
        fiducialToEye(x,y) = rot(cv::Point2i(x,y));
      }
    }
    fiducialToEye(3,0) = tvecs[i](0);
    fiducialToEye(3,1) = tvecs[i](1);
    fiducialToEye(3,2) = tvecs[i](2);
    fiducialToEye(3,3) = 1.0f;

    const Matrix4f eyeToWorld = pose.GetInvM();
    const Matrix4f fiducialToWorld = eyeToWorld * fiducialToEye;

    ORUtils::SE3Pose fiducialPose;
    fiducialPose.SetInvM(fiducialToWorld);
    fiducialPoses.push_back(fiducialPose);
  }

  return fiducialPoses;
}

boost::optional<Vector3f> ArUcoFiducialDetector::pick_corner(const cv::Point2f& corner, const VoxelRenderState_CPtr& renderState) const
{
  const int width = renderState->raycastResult->noDims.x, height = renderState->raycastResult->noDims.y;
  Vector2i p((int)CLAMP(ROUND(corner.x), 0, width - 1), (int)CLAMP(ROUND(corner.y), 0, height - 1));

  if(!m_picker) m_picker = PickerFactory::make_picker(ITMLibSettings::DEVICE_CUDA); // FIXME: Correct device type.

  static boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > pickPointFloatMB = MemoryBlockFactory::instance().make_block<Vector3f>(1);
  bool pickPointFound = m_picker->pick(p.x, p.y, renderState.get(), *pickPointFloatMB);
  if(!pickPointFound) return boost::none;

  // FIXME: This is pretty much the same as PickingSelector::get_position().
  pickPointFloatMB->UpdateHostFromDevice();
  const float voxelSize = 0.005f; // FIXME: Get this from the scene params.
  const Vector3f& pickPoint = *pickPointFloatMB->GetData(MEMORYDEVICE_CPU);
  return pickPoint * voxelSize;
}

}
