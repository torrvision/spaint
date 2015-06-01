/**
 * spaint: ImageProcessor_Shared.h
 */

#ifndef H_SPAINT_IMAGEPROCESSOR_SHARED
#define H_SPAINT_IMAGEPROCESSOR_SHARED

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief Calculates the absolute difference between the corresponding pixels of two depth images, 
 *        provided both pixel values are greater than or equal to zero. If either input pixel is 
 *        less than zero then the corresponding output pixel will be set to -1.
 *
 * \param firstInput    The first pixel value.
 * \param secondInput   The second pixel value.
 * \param output        The location into which to write the computed absolute difference.
 */
_CPU_AND_GPU_CODE_
inline void calculate_pixel_depth_difference(float firstInput, float secondInput, float *output)
{
  *output = firstInput >= 0 && secondInput >= 0 ? fabs(firstInput - secondInput) : -1.0f;
}

/**
 * \brief Shades a pixel on comparison.
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_on_comparison(float *output, float input, float comparator, ImageProcessor::ComparisonOperator comparisonOperator, float value)
{
  switch(comparisonOperator)
  {
    case ImageProcessor::CO_GREATER:
    {
      if(input > comparator) *output = value;
      else *output = input;
      break;
    }
    case ImageProcessor::CO_LESS:
    {
      if(input < comparator) *output = value;
      else *output = input;
      break;
    }
    default:
    {
      // This should never happen.
      //throw std::runtime_error("Unknown comparison type");
      printf("Unknown comparison type");
      break;
    }
  }
}

}

#endif
