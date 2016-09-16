/**
 * spaint: ImageProcessor_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_IMAGEPROCESSOR_CUDA
#define H_SPAINT_IMAGEPROCESSOR_CUDA

#include "../interface/ImageProcessor.h"

namespace spaint {

/**
 * \brief An instance of this class may be used to apply image processing algorithms to images using CUDA.
 */
class ImageProcessor_CUDA : public ImageProcessor
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void calculate_depth_difference(const ITMFloatImage_CPtr& firstInputImage, const ITMFloatImage_CPtr& secondInputImage, const AFArray_Ptr& outputImage) const;

  /** Override */
  virtual void copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMFloatImage_Ptr& outputImage) const;

  /** Override */
  virtual void copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMUCharImage_Ptr& outputImage) const;

  /** Override */
  virtual void copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMUChar4Image_Ptr& outputImage) const;

  /** Override */
  virtual void copy_itm_to_af(const ITMFloatImage_CPtr& inputImage, const AFArray_Ptr& outputImage) const;

  /** Override */
  virtual void copy_itm_to_af(const ITMUCharImage_CPtr& inputImage, const AFArray_Ptr& outputImage) const;

  /** Override */
  virtual void copy_itm_to_af(const ITMUChar4Image_CPtr& inputImage, const AFArray_Ptr& outputImage) const;

  /** Override */
  virtual void set_on_threshold(const ITMFloatImage_CPtr& inputImage, ComparisonOperator op, float threshold, float value, const ITMFloatImage_Ptr& outputImage) const;
};

}

#endif
