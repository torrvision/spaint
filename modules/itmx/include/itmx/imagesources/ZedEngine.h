/**
 * itmx: ZedEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_ZEDENGINE
#define H_ITMX_ZEDENGINE

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced when including the StereoLabs headers.
  #pragma warning(disable:4056)
#endif

#include <sl/Camera.hpp>

#ifdef _MSC_VER
  // Reenable the suppressed warnings for the rest of the translation unit.
  #pragma warning(default:4056)
#endif

#include <InputSource/ImageSourceEngine.h>

#include "../base/ITMImagePtrTypes.h"

namespace sl {

typedef boost::shared_ptr<Camera> Camera_Ptr;
typedef boost::shared_ptr<const Camera> Camera_CPtr;

}

namespace itmx {

/**
 * \brief An instance of this class can be used to yield RGB-D images that have been obtained from a Zed camera.
 */
class ZedEngine : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The intrinsic calibration parameters for the camera. */
  ITMLib::ITMRGBDCalib m_calib;

  /** The Zed camera handle. */
  sl::Camera_Ptr m_camera;

  /** A temporary image in which to store the current coloru input. */
  boost::shared_ptr<sl::Mat> m_colourImage;

  /** A temporary image in which to store the current depth input. */
  boost::shared_ptr<sl::Mat> m_depthImage;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Zed engine.
   */
  ZedEngine();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  ZedEngine(const ZedEngine&);
  ZedEngine& operator=(const ZedEngine&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual ITMLib::ITMRGBDCalib getCalib() const;

  /** Override */
  virtual Vector2i getDepthImageSize() const;

  /** Override */
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize() const;

  /** Override */
  virtual bool hasImagesNow() const;

  /** Override */
  virtual bool hasMoreImages() const;

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

}

#endif
