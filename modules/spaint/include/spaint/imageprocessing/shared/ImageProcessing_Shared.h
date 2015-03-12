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
  if((firstInput < 0) || (secondInput < 0))
  {
    *destination = -1.0f;
  }
  else
  {
    *destination = fabs(firstInput - secondInput);
  }
}

int column_major_index_from_row_major_index(int rowMajorIndex, int width, int height)
{
  int row = rowMajorIndex / width;
  int col = rowMajorIndex % width;
  return col * height + row;
}
}

#endif
