/**
 * itmx: ZedCamera.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_ZEDCAMERA
#define H_ITMX_ZEDCAMERA

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced when including the StereoLabs headers.
  #pragma warning(disable:4056)
#endif

#include <sl/Camera.hpp>

#ifdef _MSC_VER
  // Reenable the suppressed warnings for the rest of the translation unit.
  #pragma warning(default:4056)
#endif

#include <ITMLib/Objects/Camera/ITMRGBDCalib.h>
#include <ITMLib/Objects/Tracking/ITMTrackingState.h>

#include <ORUtils/SE3Pose.h>

#include "../base/ITMImagePtrTypes.h"

namespace sl {

typedef boost::shared_ptr<Camera> Camera_Ptr;
typedef boost::shared_ptr<const Camera> Camera_CPtr;

}

namespace itmx {

/**
 * \brief An instance of this class can be used to get RGB-D images and camera poses from a Zed camera.
 */
class ZedCamera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The calibration parameters for the camera. */
  ITMLib::ITMRGBDCalib m_calib;

  /** The Zed camera handle. */
  sl::Camera_Ptr m_camera;

  /** The most recent colour image retrieved from the Zed camera. */
  boost::shared_ptr<sl::Mat> m_colourImage;

  /** The most recent depth image retrieved from the Zed camera. */
  boost::shared_ptr<sl::Mat> m_depthImage;

  /** Whether or not the next attempt to get RGB-D images from the camera should trigger a grab. */
  mutable bool m_newImagesNeeded;

  /** Whether or not the next attempt to get the camera pose should trigger a grab. */
  mutable bool m_newPoseNeeded;

  /** The most recent tracking state retrieved from the Zed camera. */
  boost::shared_ptr<std::pair<ORUtils::SE3Pose,ITMLib::ITMTrackingState::TrackingResult> > m_trackingState;

  //#################### SINGLETON IMPLEMENTATION ####################
private:
  /**
   * \brief Constructs a ZedCamera instance.
   */
  ZedCamera();

public:
  /**
   * \brief Gets the singleton instance.
   *
   * \return  The singleton instance.
   */
  static boost::shared_ptr<ZedCamera>& instance();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the calibration parameters for the Zed camera.
   *
   * \return  The calibration parameters for the Zed camera.
   */
  const ITMLib::ITMRGBDCalib& get_calib() const;

  /**
   * \brief Gets the size of the depth images being produced by the Zed camera.
   *
   * \return  The size of the depth images being produced by the Zed camera.
   */
  Vector2i get_depth_image_size() const;

  /**
   * \brief Gets the next RGB-D pair from the Zed camera.
   *
   * \param rgb   An image into which to copy the next RGB image from the Zed camera.
   * \param depth An image into which to copy the next depth image from the Zed camera.
   */
  void get_images(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /**
   * \brief Gets the size of the colour images being produced by the Zed camera.
   *
   * \return  The size of the colour images being produced by the Zed camera.
   */
  Vector2i get_rgb_image_size() const;

  /**
   * \brief Gets the next tracking state from the Zed camera.
   *
   * \param trackingState A location into which to copy the next tracking state from the Zed camera.
   */
  void get_tracking_state(ITMLib::ITMTrackingState *trackingState);

  /**
   * \brief Gets whether or not the Zed camera is ready to yield an RGB-D frame.
   *
   * \return  true, if the Zed camera is ready to yield an RGB-D frame, or false otherwise.
   */
  bool has_images_now() const;

  /**
   * \brief Gets whether or not the Zed camera may still have more RGB-D frames to yield.
   *
   * \return  true, if the Zed camera may still have more RGB-D frames to yield, or false otherwise.
   */
  bool has_more_images() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the size of the images being produced by the Zed camera.
   *
   * \return  The size of the images being produced by the Zed camera.
   */
  Vector2i get_image_size() const;

  /**
   * \brief Attempts to grab the next frame of data from the Zed camera.
   *
   * \return  true, if a frame was successfully grabbed, or false otherwise.
   */
  bool grab_frame() const;

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Destroys the specified Zed camera.
   *
   * \param camera  The Zed camera to destroy.
   */
  static void destroy_camera(sl::Camera *camera);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ZedCamera> ZedCamera_Ptr;
typedef boost::shared_ptr<const ZedCamera> ZedCamera_CPtr;

}

#endif
