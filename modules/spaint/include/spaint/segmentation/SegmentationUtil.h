/**
 * spaint: SegmentationUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEGMENTATIONUTIL
#define H_SPAINT_SEGMENTATIONUTIL

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMImageTypes.h>

namespace spaint {

/**
 * \brief This class provides utility functions for working with segmentation masks.
 */
class SegmentationUtil
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMUCharImage> ITMUCharImage_CPtr;
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Applies a binary mask to an image.
   *
   * \param mask  The binary mask.
   * \param image The image to which to apply it.
   * \return      A masked version of the input image.
   */
  static ITMUChar4Image_Ptr apply_mask(const ITMUCharImage_CPtr& mask, const ITMUChar4Image_CPtr& image);
};

}

#endif
