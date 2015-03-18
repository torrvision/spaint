/**
 * spaint: ImageProcessing_Shared.h
 */

#ifndef H_SPAINT_IMAGEPROCESSING_SHARED
#define H_SPAINT_IMAGEPROCESSING_SHARED

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief Shades a pixel with the absolute difference between two other images if the values in the two other images are greater than or equal to zero.
 *
 * \param destination   The location into which to write the computed absolute difference.
 * \param firstInput    The first value.
 * \param secondInput   The second value.
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_absolute_difference(float *destination, float firstInput, float secondInput) 
{
  if((firstInput < 0) || (secondInput < 0))
  {
    *destination = -1.0f;
  }
  else
  {
    *destination = fabs(firstInput - secondInput);
  }
}
}

#endif
