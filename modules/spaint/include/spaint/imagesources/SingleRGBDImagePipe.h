/**
 * spaint: SingleRGBDImagePipe.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SINGLERGBDIMAGEPIPE
#define H_SPAINT_SINGLERGBDIMAGEPIPE

#include <boost/optional.hpp>

#include <InputSource/ImageSourceEngine.h>

#include "../util/ITMImagePtrTypes.h"

namespace spaint {

/**
 * \brief TODO
 */
class SingleRGBDImagePipe : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  boost::optional<ITMLib::ITMRGBDCalib> m_calib;

  /** TODO */
  ITMShortImage_CPtr m_rawDepthImage;

  /** TODO */
  ITMUChar4Image_CPtr m_rgbImage;

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
  void set_calib(const ITMLib::ITMRGBDCalib& calib);

  /**
   * \brief TODO
   */
  void set_images(const ITMUChar4Image_CPtr& rgbImage, const ITMShortImage_CPtr& rawDepthImage);
};

}

#endif
