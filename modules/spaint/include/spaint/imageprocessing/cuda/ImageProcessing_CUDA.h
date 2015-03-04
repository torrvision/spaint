/**
 * spaint: ImageProcessing_CUDA.h
 */

#ifndef H_SPAINT_DEPTHCALCULATOR_CUDA
#define H_SPAINT_DEPTHCALCULATOR_CUDA

#include "../interface/ImageProcessing.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to apply image processing algorithms to images with CUDA.
 */
class ImageProcessing_CUDA : public ImageProcessing
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override. */
  virtual void absolute_difference_calculator(ITMFloatImage *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const;

  /** Override. */
  //virtual void binary_threshold_calculator(ITMFloatImage *outputImage, ITMFloatImage *inputImage, float threshold, float maxBinaryValue) const;
};

}

#endif
