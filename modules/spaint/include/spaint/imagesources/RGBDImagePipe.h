/**
 * spaint: RGBDImagePipe.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_RGBDIMAGEPIPE
#define H_SPAINT_RGBDIMAGEPIPE

#include <InputSource/ImageSourceEngine.h>

namespace spaint {

/**
 * \brief TODO
 */
class RGBDImagePipe : public InputSource::ImageSourceEngine
{
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
};

}

#endif
