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
   * \brief Tests the value of a pixel in an input image against a threshold using the specified comparison operator,
   *        and either writes a specified value to the corresponding pixel in the output image (if the test is passed),
   *        or copies the value of the input pixel across (otherwise).
   *
   * \param input     The input pixel value.
   * \param op        The comparison operator.
   * \param threshold The value against which to compare the pixel value.
   * \param value     The value to which to set the pixel in the output image when the input pixel passes the test.
   * \param output    The location into which to write the output pixel value.
   */
_CPU_AND_GPU_CODE_
inline void set_pixel_on_threshold(float input, ImageProcessor::ComparisonOperator op, float threshold, float value, float *output)
{
  switch(op)
  {
    case ImageProcessor::CO_GREATER:
    {
      if(input > threshold) *output = value;
      else *output = input;
      break;
    }
    case ImageProcessor::CO_LESS:
    {
      if(input < threshold) *output = value;
      else *output = input;
      break;
    }
    default:
    {
      // This should never happen.
      break;
    }
  }
}

}

#endif
