/**
 * spaint: ImageProcessor_CPU.cpp
 */

#include "imageprocessing/cpu/ImageProcessor_CPU.h"

#include "imageprocessing/shared/ImageProcessor_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessor_CPU::absolute_difference_calculator(ITMFloatImage *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const
{
  ImageProcessor::check_image_size_equal(outputImage, firstInputImage);
  ImageProcessor::check_image_size_equal(secondInputImage, firstInputImage);

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

void ImageProcessor_CPU::absolute_difference_calculator(af::array *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const
{
  ImageProcessor::check_image_size_equal(outputImage, firstInputImage);
  ImageProcessor::check_image_size_equal(secondInputImage, firstInputImage);

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
    int locIdcm = ImageProcessor::column_major_index_from_row_major_index(locId, width, height);
    shade_pixel_absolute_difference(&output[locIdcm], first[locId], second[locId]);
  }
}

}
