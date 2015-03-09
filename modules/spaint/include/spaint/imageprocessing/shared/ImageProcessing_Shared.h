/**
 * spaint: ImageProcessing_Shared.h
 */

#ifndef H_SPAINT_IMAGEPROCESSING_SHARED
#define H_SPAINT_IMAGEPROCESSING_SHARED

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void shade_pixel_absolute_difference(float *destination, float firstInput, float secondInput) 
{
  /*if((firstInput < 0) || (secondInput < 0))
  {
    *destination = -1.0f;
  }
  else
  {
    *destination = fabs(firstInput - secondInput);
  }*/
  *destination = 0.005;
}

}

#endif
