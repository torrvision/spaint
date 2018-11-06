/**
 * spaint: ArUcoFiducialDetector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/ArUcoFiducialDetector.h"

#include <cmath>

#include <boost/lexical_cast.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <ITMLib/Objects/Camera/ITMIntrinsics.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
using namespace ITMLib;

#include <ORUtils/ProjectionUtils.h>

#include <itmx/ocv/OpenCVUtil.h>
#include <itmx/picking/PickerFactory.h>
using namespace itmx;

#include <orx/base/MemoryBlockFactory.h>
using namespace orx;

namespace spaint {

//#################### CONSTRUCTORS ####################

ArUcoFiducialDetector::ArUcoFiducialDetector(const SpaintVoxelScene_CPtr& scene, const Settings_CPtr& settings, PoseEstimationMode poseEstimationMode)
: m_poseEstimationMode(poseEstimationMode),
  m_scene(scene),
  m_settings(settings),
  m_voxelVisualisationEngine(ITMVisualisationEngineFactory::MakeVisualisationEngine<SpaintVoxel,ITMVoxelIndex>(settings->deviceType))
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::map<std::string,FiducialMeasurement>
ArUcoFiducialDetector::detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& depthPose) const
{
  std::map<std::string,FiducialMeasurement> result;

  // Convert the current colour input image to OpenCV format.
  const ORUChar4Image *rgb = view->rgb;
  rgb->UpdateHostFromDevice();
  cv::Mat3b rgbImage = OpenCVUtil::make_rgb_image(rgb->GetData(MEMORYDEVICE_CPU), rgb->noDims.x, rgb->noDims.y);

  // Detect any ArUco fiducials that are visible.
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  std::vector<std::vector<cv::Point2f> > corners;
  std::vector<int> ids;
  cv::aruco::detectMarkers(rgbImage, dictionary, corners, ids);

#if 0
  // Visualise the detected fiducials for debugging purposes.
  cv::Mat3b markerImage = rgbImage.clone();
  cv::aruco::drawDetectedMarkers(markerImage, corners, ids);
  cv::imshow("Detected Markers", markerImage);
#endif

  // Construct the fiducial measurements.
  std::vector<boost::optional<FiducialMeasurement> > measurements;
  switch(m_poseEstimationMode)
  {
    case PEM_COLOUR:
      measurements = construct_measurements_from_colour(ids, corners, view, depthPose);
      break;
    case PEM_DEPTH:
      measurements = construct_measurements_from_depth(ids, corners, view, depthPose);
      break;
    case PEM_RAYCAST:
      measurements = construct_measurements_from_raycast(ids, corners, view, depthPose);
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Unknown fiducial pose estimation mode");
  }

  for(size_t i = 0, size = ids.size(); i < size; ++i)
  {
    if(!measurements[i]) continue;
    result.insert(std::make_pair(boost::lexical_cast<std::string>(ids[i]), *measurements[i]));
  }

  return result;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

std::vector<boost::optional<FiducialMeasurement> >
ArUcoFiducialDetector::construct_measurements_from_colour(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                          const View_CPtr& view, const ORUtils::SE3Pose& depthPose) const
{
  std::vector<boost::optional<FiducialMeasurement> > measurements;

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
        fiducialToEye(x,y) = static_cast<float>(rot(cv::Point2i(x,y)));
      }
    }
    fiducialToEye(3,0) = static_cast<float>(tvecs[i](0));
    fiducialToEye(3,1) = static_cast<float>(tvecs[i](1));
    fiducialToEye(3,2) = static_cast<float>(tvecs[i](2));
    fiducialToEye(3,3) = 1.0f;

    const Matrix4f eyeToWorld = depthPose.GetInvM();
    const Matrix4f fiducialToWorld = eyeToWorld * fiducialToEye;

    ORUtils::SE3Pose fiducialPoseWorld;
    fiducialPoseWorld.SetInvM(fiducialToWorld);
    measurements.push_back(FiducialMeasurement(boost::lexical_cast<std::string>(ids[i]), boost::none, fiducialPoseWorld));
  }

  return measurements;
}

std::vector<boost::optional<FiducialMeasurement> >
ArUcoFiducialDetector::construct_measurements_from_depth(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                         const View_CPtr& view, const ORUtils::SE3Pose& depthPose) const
{
  std::vector<boost::optional<FiducialMeasurement> > measurements;

  // Make sure that the live depth image is available on the CPU.
  view->depth->UpdateHostFromDevice();

  for(size_t i = 0, size = corners.size(); i < size; ++i)
  {
    boost::optional<ORUtils::SE3Pose> fiducialPoseEye = make_pose_from_corners(
      pick_corner_from_depth(corners[i][3], view),
      pick_corner_from_depth(corners[i][2], view),
      pick_corner_from_depth(corners[i][0], view)
    );

    measurements.push_back(make_measurement_from_eye_pose(
      boost::lexical_cast<std::string>(ids[i]), fiducialPoseEye, depthPose
    ));
  }

  return measurements;
}

std::vector<boost::optional<FiducialMeasurement> >
ArUcoFiducialDetector::construct_measurements_from_raycast(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f> >& corners,
                                                           const View_CPtr& view, const ORUtils::SE3Pose& depthPose) const
{
  std::vector<boost::optional<FiducialMeasurement> > measurements;

  if(!m_renderState) m_renderState.reset(ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(view->rgb->noDims, m_scene->sceneParams, m_settings->GetMemoryType()));

  // Raycast the scene from the pose of the colour camera.
  const ORUtils::SE3Pose rgbPose = view->calib.trafo_rgb_to_depth.calib_inv * depthPose.GetM();
  const ITMIntrinsics& rgbIntrinsics = view->calib.intrinsics_rgb;
  m_voxelVisualisationEngine->FindVisibleBlocks(m_scene.get(), &rgbPose, &rgbIntrinsics, m_renderState.get());
  m_voxelVisualisationEngine->CreateExpectedDepths(m_scene.get(), &rgbPose, &rgbIntrinsics, m_renderState.get());
  m_voxelVisualisationEngine->FindSurface(m_scene.get(), &rgbPose, &rgbIntrinsics, m_renderState.get());

  // Try to determine the 3D points in world space corresponding to each fiducial's corners in the
  // live colour image by looking them up in the raycast. If successful, use them to determine poses
  // for the fiducial in both world and eye space, and record a fiducial measurement.
  for(size_t i = 0, size = corners.size(); i < size; ++i)
  {
    boost::optional<ORUtils::SE3Pose> fiducialPoseWorld = make_pose_from_corners(
      pick_corner_from_raycast(corners[i][3]),
      pick_corner_from_raycast(corners[i][2]),
      pick_corner_from_raycast(corners[i][0])
    );

    measurements.push_back(make_measurement_from_world_pose(
      boost::lexical_cast<std::string>(ids[i]), fiducialPoseWorld, depthPose
    ));
  }

  return measurements;
}

boost::optional<Vector3f> ArUcoFiducialDetector::pick_corner_from_depth(const cv::Point2f& corner, const View_CPtr& view) const
{
  // FIXME: I'm currently assuming that there is an identity mapping between the depth and colour cameras - in general, this won't be the case.
  const int width = view->depth->noDims.x, height = view->depth->noDims.y;
  const int ux = (int)CLAMP(ROUND(corner.x), 0, width - 1), uy = (int)CLAMP(ROUND(corner.y), 0, height - 1);
  const int locId = uy * width + ux;
  const float depth = view->depth->GetData(MEMORYDEVICE_CPU)[locId];

  const float EPSILON = 1e-3f;
  if(fabs(depth + 1) > EPSILON) // i.e. if(depth != -1)
  {
    return unproject(ux, uy, depth, view->calib.intrinsics_d.projectionParamsSimple.all);
  }
  else return boost::none;
}

boost::optional<Vector3f> ArUcoFiducialDetector::pick_corner_from_raycast(const cv::Point2f& corner) const
{
  const int width = m_renderState->raycastResult->noDims.x, height = m_renderState->raycastResult->noDims.y;
  const Vector2i p((int)CLAMP(ROUND(corner.x), 0, width - 1), (int)CLAMP(ROUND(corner.y), 0, height - 1));

  if(!m_picker) m_picker = PickerFactory::make_picker(m_settings->deviceType);

  static boost::shared_ptr<ORUtils::MemoryBlock<Vector3f> > pickPointFloatMB = MemoryBlockFactory::instance().make_block<Vector3f>(1);
  bool pickPointFound = m_picker->pick(p.x, p.y, m_renderState.get(), *pickPointFloatMB);
  if(!pickPointFound) return boost::none;

  return Picker::get_positions<Vector3f>(*pickPointFloatMB, m_settings->sceneParams.voxelSize)[0];
}

}
