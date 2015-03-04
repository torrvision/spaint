/**
 * spaint: ImageProcessing_CPU.cpp
 */

#include "imageprocessing/cpu/ImageProcessing_CPU.h"

#include "imageprocessing/shared/ImageProcessing_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessing_CPU::absolute_difference_calculator(ITMFloatImage *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const
{
  ImageProcessing::check_image_size_equal(outputImage, firstInputImage);
  ImageProcessing::check_image_size_equal(secondInputImage, firstInputImage);

  int imgSize = outputImage->noDims.x * outputImage->noDims.y;
  float *output = outputImage->GetData(MEMORYDEVICE_CPU);
  float *first = firstInputImage->GetData(MEMORYDEVICE_CPU);
  float *second = secondInputImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < imgSize; ++locId)
  {
    shade_pixel_absolute_difference(&output[locId], first[locId], second[locId]);
  }
}

/*void ImageProcessing_CPU::binary_threshold_calculator(ITMFloatImage *outputImage, ITMFloatImage *inputImage, float threshold, float maxBinaryValue) const
{
  ImageProcessing::check_image_size_equal(outputImage, inputImage);

  int imgSize = outputImage->noDims.x * outputImage->noDims.y;
  float *output = outputImage->GetData(MEMORYDEVICE_CPU);
  float *input = inputImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < imgSize; ++locId)
  {
    shade_pixel_binary_threshold(&output[locId], first[locId], threshold, maxBinaryValue);
  }
}*/

}
