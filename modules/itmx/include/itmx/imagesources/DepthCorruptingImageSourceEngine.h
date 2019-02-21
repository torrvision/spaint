/**
 * itmx: DepthCorruptingImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#ifndef H_ITMX_DEPTHCORRUPTINGIMAGESOURCEENGINE
#define H_ITMX_DEPTHCORRUPTINGIMAGESOURCEENGINE

#include <orx/base/ORImagePtrTypes.h>

#include <tvgutil/numbers/RandomNumberGenerator.h>

#include "../base/ITMObjectPtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to yield RGB-D images in which the depth has been corrupted in various ways.
 */
class DepthCorruptingImageSourceEngine : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The sigma of the Gaussian to use when corrupting the depth with zero-mean, depth-dependent Gaussian noise (0 = disabled). */
  float m_depthNoiseSigma;

  /** The image source from which to obtain the uncorrupted images. */
  ImageSourceEngine_Ptr m_innerSource;

  /** A mask indicating which pixels have missing depth. */
  ORBoolImage_Ptr m_missingDepthMask;

  /** The random number generator. */
  tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a depth-corrupting image source engine.
   *
   * \param innerSource           The image source from which to obtain the uncorrupted images.
   * \param missingDepthFraction  The fraction of the depth images to zero out (in the range [0,1]).
   * \param depthNoiseSigma       The sigma of the Gaussian to use when corrupting the depth with zero-mean, depth-dependent Gaussian noise (0 = disabled).
   */
  DepthCorruptingImageSourceEngine(ImageSourceEngine *innerSource, double missingDepthFraction, float depthNoiseSigma);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual ITMLib::ITMRGBDCalib getCalib() const;

  /** Override */
  virtual Vector2i getDepthImageSize() const;

  /** Override */
  virtual void getImages(ORUChar4Image *rgb, ORShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize() const;

  /** Override */
  virtual bool hasMoreImages() const;
};

}

#endif
