/**
 * spaint: SingleRGBDImagePipe.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SINGLERGBDIMAGEPIPE
#define H_SPAINT_SINGLERGBDIMAGEPIPE

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMObjectPtrTypes.h>

namespace spaint {

/**
 * \brief An instance of this class represents a pipe to which individual RGB-D images can be
 *        written so as to feed them to a SLAM component.
 */
class SingleRGBDImagePipe : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The intrinsic calibration parameters for the camera producing the RGB-D images being fed through the pipe. */
  ITMLib::ITMRGBDCalib m_calib;

  /** The current depth image being fed through the pipe. */
  ITMShortImage_CPtr m_depthImage;

  /** The size of depth image being fed through the pipe. */
  Vector2i m_depthImageSize;

  /** The current RGB image being fed through the pipe. */
  ITMUChar4Image_CPtr m_rgbImage;

  /** The size of RGB image being fed through the pipe. */
  Vector2i m_rgbImageSize;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a pipe to which individual RGB-D images can be written so as to feed them to a SLAM component.
   *
   * \param imageSourceEngine The engine that will be providing the RGB-D images that will be fed through the pipe.
   */
  explicit SingleRGBDImagePipe(const ImageSourceEngine_CPtr& imageSourceEngine);

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
  virtual bool hasMoreImages() const;

  /**
   * \brief Sets the current RGB and depth images to feed through the pipe.
   *
   * \param rgbImage    The current RGB image to feed through the pipe.
   * \param depthImage  The current depth image to feed through the pipe.
   */
  void set_images(const ITMUChar4Image_CPtr& rgbImage, const ITMShortImage_CPtr& depthImage);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SingleRGBDImagePipe> SingleRGBDImagePipe_Ptr;
typedef boost::shared_ptr<const SingleRGBDImagePipe> SingleRGBDImagePipe_CPtr;

}

#endif
