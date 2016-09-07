/**
 * spaint: SingleRGBDImagePipe.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SINGLERGBDIMAGEPIPE
#define H_SPAINT_SINGLERGBDIMAGEPIPE

#include <boost/optional.hpp>

#include "../util/ITMImagePtrTypes.h"
#include "../util/ITMObjectPtrTypes.h"

namespace spaint {

/**
 * \brief TODO
 */
class SingleRGBDImagePipe : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  ITMLib::ITMRGBDCalib m_calib;

  /** TODO */
  ITMShortImage_CPtr m_depthImage;

  /** TODO */
  Vector2i m_depthImageSize;

  /** TODO */
  ITMUChar4Image_CPtr m_rgbImage;

  /** TODO */
  Vector2i m_rgbImageSize;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
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
   * \brief TODO
   */
  void set_images(const ITMUChar4Image_CPtr& rgbImage, const ITMShortImage_CPtr& rawDepthImage);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SingleRGBDImagePipe> SingleRGBDImagePipe_Ptr;
typedef boost::shared_ptr<const SingleRGBDImagePipe> SingleRGBDImagePipe_CPtr;

}

#endif
