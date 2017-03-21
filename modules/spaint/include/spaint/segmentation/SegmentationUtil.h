/**
 * spaint: SegmentationUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEGMENTATIONUTIL
#define H_SPAINT_SEGMENTATIONUTIL

#include <boost/mpl/identity.hpp>

#include <tvgutil/itm/ITMImagePtrTypes.h>

namespace spaint {

/**
 * \brief This class provides utility functions for working with segmentation masks.
 */
class SegmentationUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Applies a binary mask to an image.
   *
   * This version of the function is needed to assist the compiler with type deduction.
   *
   * \param mask            The binary mask.
   * \param image           The image to which to apply it.
   * \param backgroundValue The value to use for background pixels in the masked image.
   * \return                A masked version of the input image.
   */
  template <typename T>
  static boost::shared_ptr<ORUtils::Image<T> > apply_mask(const ITMUCharImage_CPtr& mask, const boost::shared_ptr<ORUtils::Image<T> >& image,
                                                          const typename boost::mpl::identity<T>::type& backgroundValue)
  {
    return apply_mask(mask, boost::shared_ptr<const ORUtils::Image<T> >(image), backgroundValue);
  }

  /**
   * \brief Applies a binary mask to an image.
   *
   * \param mask            The binary mask.
   * \param image           The image to which to apply it.
   * \param backgroundValue The value to use for background pixels in the masked image.
   * \return                A masked version of the input image.
   */
  template <typename T>
  static boost::shared_ptr<ORUtils::Image<T> > apply_mask(const ITMUCharImage_CPtr& mask, const boost::shared_ptr<const ORUtils::Image<T> >& image,
                                                          const typename boost::mpl::identity<T>::type& backgroundValue)
  {
    boost::shared_ptr<ORUtils::Image<T> > maskedImage(new ORUtils::Image<T>(image->noDims, true, true));

    const uchar *maskPtr = mask->GetData(MEMORYDEVICE_CPU);
    const T *imagePtr = image->GetData(MEMORYDEVICE_CPU);
    T *maskedImagePtr = maskedImage->GetData(MEMORYDEVICE_CPU);
    int pixelCount = static_cast<int>(image->dataSize);

  #ifdef WITH_OPENMP
    #pragma omp parallel for
  #endif
    for(int i = 0; i < pixelCount; ++i)
    {
      maskedImagePtr[i] = maskPtr[i] ? imagePtr[i] : backgroundValue;
    }

    return maskedImage;
  }

  /**
   * \brief Inverts a binary mask.
   *
   * \param mask  The mask to invert.
   * \return      An inverted version of the mask.
   */
  static ITMUCharImage_Ptr invert_mask(const ITMUCharImage_CPtr& mask);
};

}

#endif
