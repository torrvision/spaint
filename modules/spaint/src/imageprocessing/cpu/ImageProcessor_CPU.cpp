/**
 * spaint: ImageProcessor_CPU.cpp
 */

#include "imageprocessing/cpu/ImageProcessor_CPU.h"

#include "imageprocessing/shared/ImageProcessor_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessor_CPU::calculate_depth_difference(const ITMFloatImage_CPtr& firstInputImage, const ITMFloatImage_CPtr& secondInputImage, const AFImage_Ptr& outputImage) const
{
  check_image_size_equal(firstInputImage, secondInputImage);
  check_image_size_equal(firstInputImage, outputImage);

  const float *firstInput = firstInputImage->GetData(MEMORYDEVICE_CPU);
  const float *secondInput = secondInputImage->GetData(MEMORYDEVICE_CPU);
  float *output = outputImage->device<float>(); // note that using host<float>() doesn't work!

  const int height = outputImage->dims(0);
  const int width = outputImage->dims(1);
  const int imgSize = height * width;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int rowMajorIndex = 0; rowMajorIndex < imgSize; ++rowMajorIndex)
  {
    int columnMajorIndex = ImageProcessor::column_major_index_from_row_major_index(rowMajorIndex, width, height);
    calculate_pixel_depth_difference(firstInput[rowMajorIndex], secondInput[rowMajorIndex], &output[columnMajorIndex]);
  }
}

void ImageProcessor_CPU::set_on_threshold(const ITMFloatImage_CPtr& inputImage, ComparisonOperator op, float threshold, float value, const ITMFloatImage_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  const int imgSize = inputImage->noDims.x * inputImage->noDims.y;
  const float *inputData = inputImage->GetData(MEMORYDEVICE_CPU);
  float *outputData = outputImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int i = 0; i < imgSize; ++i)
  {
    set_pixel_on_threshold(inputData[i], op, threshold, value, &outputData[i]);
  }
}

}
