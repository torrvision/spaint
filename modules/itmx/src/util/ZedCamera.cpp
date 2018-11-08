/**
 * itmx: ZedCamera.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "util/ZedCamera.h"
using namespace ITMLib;

#include <stdexcept>

namespace itmx {

//#################### SINGLETON IMPLEMENTATION ####################

ZedCamera::ZedCamera(int gpuID)
: m_depthConfidenceThreshold(0.3f),
  m_newImagesNeeded(true),
  m_newPoseNeeded(true)
{
  // Construct the Zed camera.
  sl::Camera *camera = new sl::Camera;

  // Set up the initialisation parameters for the camera.
  sl::InitParameters initParams;
  initParams.camera_resolution = sl::RESOLUTION_VGA;
  initParams.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
  initParams.coordinate_units = sl::UNIT_METER;
  initParams.depth_mode = sl::DEPTH_MODE_QUALITY;
  cuCtxGetCurrent(&initParams.sdk_cuda_ctx);
  initParams.sdk_gpu_id = gpuID;

  // Try to open the camera. If we fail, throw.
  sl::ERROR_CODE err = camera->open(initParams);
  if(err != sl::SUCCESS)
  {
    delete camera;
    throw std::runtime_error("Error: Could not open Zed camera");
  }

  // Save the camera pointer and ensure that the camera will eventually be closed and deleted correctly.
  m_camera.reset(camera, destroy_camera);

  // Make the internal images into which to store images retrieved from the camera.
  m_colourImage.reset(new sl::Mat(m_camera->getResolution(), sl::MAT_TYPE_8U_C4));
  m_depthImage.reset(new sl::Mat);
  m_depthConfidenceImage.reset(new sl::Mat);

  // Retrieve the camera calibration parameters.
  sl::CalibrationParameters calibParams = m_camera->getCameraInformation().calibration_parameters;
  m_calib.intrinsics_rgb.SetFrom(
    static_cast<int>(m_colourImage->getWidth()),
    static_cast<int>(m_colourImage->getHeight()),
    calibParams.left_cam.fx, calibParams.left_cam.fy,
    calibParams.left_cam.cx, calibParams.left_cam.cy
  );
  m_calib.intrinsics_d = m_calib.intrinsics_rgb;

  // Enable tracking.
  sl::TrackingParameters trackingParams;
  m_camera->enableTracking(trackingParams);
}

ZedCamera_Ptr& ZedCamera::instance()
{
  static ZedCamera_Ptr s_instance(new ZedCamera);
  return s_instance;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const ITMRGBDCalib& ZedCamera::get_calib() const
{
  return m_calib;
}

Vector2i ZedCamera::get_depth_image_size() const
{
  return get_image_size();
}

void ZedCamera::get_images(ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  // If needed, try to grab the next frame.
  if(m_newImagesNeeded)
  {
    bool succeeded = grab_frame();
    if(!succeeded) return;
  }

  // Retrieve the colour, depth and depth confidence images from the camera and store them in the internal images.
  m_camera->retrieveImage(*m_colourImage, sl::VIEW_LEFT);
  m_camera->retrieveMeasure(*m_depthImage, sl::MEASURE_DEPTH);
  m_camera->retrieveMeasure(*m_depthConfidenceImage, sl::MEASURE_CONFIDENCE);

  // Copy the internal colour image into the output colour image, converting its format from BGRA to RGBA in the process.
  {
    unsigned char *dest = reinterpret_cast<unsigned char*>(rgb->GetData(MEMORYDEVICE_CPU));
    const unsigned char *src = m_colourImage->getPtr<sl::uchar1>();
    for(size_t i = 0, size = m_colourImage->getWidthBytes() * m_colourImage->getHeight(); i < size; i += 4)
    {
      dest[i+0] = src[i+2];
      dest[i+1] = src[i+1];
      dest[i+2] = src[i+0];
      dest[i+3] = src[i+3];
    }
  }

  // Copy the internal depth image into the output depth image, filtering based on confidence and converting its format from float to short in the process.
  {
    const float confidenceScalingFactor = 0.01f;
    short *dest = rawDepth->GetData(MEMORYDEVICE_CPU);
    const float *src = m_depthImage->getPtr<float>();
    const float *depthConfidence = m_depthConfidenceImage->getPtr<float>();
    for(size_t i = 0, size = m_depthImage->getWidth() * m_depthImage->getHeight(); i < size; ++i)
    {
      const float depth = src[i];
      const float confidence = 1.0f - depthConfidence[i] * confidenceScalingFactor;
      dest[i] = isValidMeasure(depth) && confidence >= m_depthConfidenceThreshold
        ? (short)(CLAMP(ROUND(depth / m_calib.disparityCalib.GetParams()[0]), 0, std::numeric_limits<short>::max()))
        : 0;
    }
  }

  m_newImagesNeeded = true;
}

Vector2i ZedCamera::get_rgb_image_size() const
{
  return get_image_size();
}

void ZedCamera::get_tracking_state(ITMTrackingState *trackingState)
{
  // If needed, try to grab the next frame.
  if(m_newPoseNeeded)
  {
    bool succeeded = grab_frame();
    if(!succeeded)
    {
      trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;
      return;
    }
  }

  // Retrieve the Zed tracking state from the camera.
  sl::Pose pose;
  sl::TRACKING_STATE state = m_camera->getPosition(pose, sl::REFERENCE_FRAME_WORLD);
  sl::Rotation R = pose.getRotation();
  sl::Translation t = pose.getTranslation();

  // If tracking is currently succeeding, update the camera pose accordingly and report successful tracking;
  // otherwise, leave the camera pose alone and report that the tracking has failed.
  if(state == sl::TRACKING_STATE_OK)
  {
    Matrix4f M;
    M(0,0) = R(0,0); M(1,0) = R(0,1); M(2,0) = R(0,2); M(3,0) = t.x;
    M(0,1) = R(1,0); M(1,1) = R(1,1); M(2,1) = R(1,2); M(3,1) = t.y;
    M(0,2) = R(2,0); M(1,2) = R(2,1); M(2,2) = R(2,2); M(3,2) = t.z;
    M(0,3) = 0.0f;   M(1,3) = 0.0f;   M(2,3) = 0.0f;   M(3,3) = 1.0f;

    trackingState->pose_d->SetInvM(M);
    trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
  }
  else
  {
    trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;
  }

  m_newPoseNeeded = true;
}

bool ZedCamera::has_images_now() const
{
  return !m_newImagesNeeded || grab_frame();
}

bool ZedCamera::has_more_images() const
{
  return m_camera->isOpened();
}

void ZedCamera::set_depth_confidence_threshold(float depthConfidenceThreshold)
{
  m_depthConfidenceThreshold = depthConfidenceThreshold;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

Vector2i ZedCamera::get_image_size() const
{
  if(m_camera)
  {
    sl::Resolution imgSize = m_camera->getResolution();
    return Vector2i(static_cast<int>(imgSize.width), static_cast<int>(imgSize.height));
  }
  else return Vector2i(0,0);
}

bool ZedCamera::grab_frame() const
{
  // Set the sensing mode.
  sl::RuntimeParameters params;
  params.sensing_mode = sl::SENSING_MODE_STANDARD;

  // Attempt to grab the next frame.
  if(m_camera->grab(params) == sl::SUCCESS)
  {
    m_newImagesNeeded = m_newPoseNeeded = false;
    return true;
  }
  else return false;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void ZedCamera::destroy_camera(sl::Camera *camera)
{
  if(camera)
  {
    camera->close();
    delete camera;
  }
}

}
