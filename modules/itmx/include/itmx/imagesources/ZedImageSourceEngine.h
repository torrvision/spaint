/**
 * itmx: ZedImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_ZEDIMAGESOURCEENGINE
#define H_ITMX_ZEDIMAGESOURCEENGINE

#include <InputSource/ImageSourceEngine.h>

#include "../util/ZedCamera.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to yield RGB-D images that have been obtained from a Zed camera.
 */
class ZedImageSourceEngine : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The Zed camera from which to obtain the RGB-D images. */
  ZedCamera_Ptr m_camera;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Zed image source engine.
   *
   * \param camera  The Zed camera from which to obtain the RGB-D images.
   */
  explicit ZedImageSourceEngine(const ZedCamera_Ptr& camera);

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
};

}

#endif
