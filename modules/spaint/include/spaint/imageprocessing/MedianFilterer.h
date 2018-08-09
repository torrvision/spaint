/**
 * spaint: MedianFilterer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_MEDIANFILTERER
#define H_SPAINT_MEDIANFILTERER

#include <ORUtils/DeviceType.h>

#include "interface/ImageProcessor.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to perform median filtering on RGBA images.
 */
class MedianFilterer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The image processor. */
  ImageProcessor_CPtr m_imageProcessor;

  /** An intermediate ArrayFire image on which to actually perform the filtering. */
  mutable boost::shared_ptr<af::array> m_intermediate;

  /** The kernel width to use for median filtering. */
  unsigned int m_kernelWidth;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a median filterer.
   *
   * \param kernelWidth The kernel width to use for median filtering.
   * \param deviceType  The device on which the filterer should operate.
   */
  MedianFilterer(unsigned int kernelWidth, DeviceType deviceType);

  //#################### PUBLIC OPERATORS ####################
public:
  /**
   * \brief Performs median filtering on an RGBA input image to produce an RGBA output image.
   *
   * The median filtering will be performed using the parameters provided when the filterer was constructed.
   *
   * FIXME: Median filtering can occasionally fail due to a possible crash bug in ArrayFire. If this happens,
   *        we currently avoid throwing and instead treat this function as a no-op.
   *
   * \param input   The input image.
   * \param output  The output image.
   */
  void operator()(const ITMUChar4Image_CPtr& input, const ITMUChar4Image_Ptr& output) const;
};

}

#endif
