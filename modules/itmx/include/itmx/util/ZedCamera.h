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
 * \brief An instance of this class can be used to get RGB-D images and poses from a Zed camera.
 */
class ZedCamera
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The intrinsic calibration parameters for the camera. */
  ITMLib::ITMRGBDCalib m_calib;

  /** The Zed camera handle. */
  sl::Camera_Ptr m_camera;

  /** A temporary image in which to store the current colour input. */
  boost::shared_ptr<sl::Mat> m_colourImage;

  /** A temporary image in which to store the current depth input. */
  boost::shared_ptr<sl::Mat> m_depthImage;

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
   * \brief TODO
   */
  const ITMLib::ITMRGBDCalib& get_calib() const;

  /**
   * \brief TODO
   */
  Vector2i get_depth_image_size() const;

  /**
   * \brief TODO
   */
  void get_images(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /**
   * \brief TODO
   */
  Vector2i get_rgb_image_size() const;

  /**
   * \brief TODO
   */
  void get_tracking_state(ITMLib::ITMTrackingState *trackingState);

  /**
   * \brief TODO
   */
  bool has_images_now() const;

  /**
   * \brief TODO
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
