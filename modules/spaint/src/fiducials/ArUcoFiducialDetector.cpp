/**
 * spaint: ArUcoFiducialDetector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/ArUcoFiducialDetector.h"

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include <opencv2/aruco.hpp>

#include <ITMLib/Objects/Camera/ITMIntrinsics.h>
using namespace ITMLib;

#include "ocv/OpenCVUtil.h"
#include "util/CameraPoseConverter.h"
using namespace rigging;

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::map<std::string,Fiducial> ArUcoFiducialDetector::detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& pose) const
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
  for(size_t i = 0, size = ids.size(); i < size; ++i)
  {
    std::string id = boost::lexical_cast<std::string>(ids[i]);

    Vector3f eyePos = Vector3d(tvecs[i](0), tvecs[i](1), tvecs[i](2)).toFloat();
    Vector3f worldPos = pose.GetInvM() * eyePos.toFloat();

    fiducials.insert(std::make_pair(id, Fiducial(id, worldPos)));
  }

  return fiducials;
}

}
