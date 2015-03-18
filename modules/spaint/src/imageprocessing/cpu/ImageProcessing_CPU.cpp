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

void ImageProcessing_CPU::absolute_difference_calculator(af::array *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const
{
  ImageProcessing::check_image_size_equal(outputImage, firstInputImage);
  ImageProcessing::check_image_size_equal(secondInputImage, firstInputImage);

  int height = outputImage->dims(0);
  int width = outputImage->dims(1);
  int imgSize = height * width;
  float *output = outputImage->device<float>(); // Selecing host pointer does not work.
  float *first = firstInputImage->GetData(MEMORYDEVICE_CPU);
  float *second = secondInputImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < imgSize; ++locId)
  {
    int locIdcm = ImageProcessing::column_major_index_from_row_major_index(locId, width, height);
    shade_pixel_absolute_difference(&output[locIdcm], first[locId], second[locId]);
  }
}

}
