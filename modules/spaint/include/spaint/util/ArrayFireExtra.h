/**
 * spaint: ArrayFireExtra.h
 */

#ifndef H_SPAINT_ARRAYFIREEXTRA
#define H_SPAINT_ARRAYFIREEXTRA

#include <arrayfire.h>
#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

class ArrayFireExtra
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  static af::array ITM2AF(ITMFloatImage *infiniTAMImage)
  {
    int cols = infiniTAMImage->noDims.x;
    int rows = infiniTAMImage->noDims.y;

#ifdef WITH_CUDA
    float *infiniTAMImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CUDA);
    af::array arrayfireImage(rows, cols, infiniTAMImageDataPtr, af::afDevice); 
#else
    float *infiniTAMImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);
    af::array arrayfireImage(rows, cols, infiniTAMImageDataPtr, af::afHost); 
#endif

    return arrayfireImage;
  }

};

}

#endif

