/**
 * spaint: ImageProcessor_Shared.h
 */

#ifndef H_SPAINT_IMAGEPROCESSOR_SHARED
#define H_SPAINT_IMAGEPROCESSOR_SHARED

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief Calculates the absolute difference between the corresponding pixels of two images, 
 * if the corresponding pixel values in the two images are greater than or equal to zero.
 *
 * \param destination   The location into which to write the computed absolute difference.
 * \param firstInput    The first value.
 * \param secondInput   The second value.
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_absolute_difference(float *destination, float firstInput, float secondInput)
{
  *destination = firstInput >= 0 && secondInput >= 0 ? fabs(firstInput - secondInput) : -1.0f;
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
