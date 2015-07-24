/**
 * spaint: ImageProcessor_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "imageprocessing/cpu/ImageProcessor_CPU.h"

#include "imageprocessing/shared/ImageProcessor_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessor_CPU::calculate_depth_difference(const ITMFloatImage_CPtr& firstInputImage, const ITMFloatImage_CPtr& secondInputImage, const AFArray_Ptr& outputImage) const
{
  check_image_size_equal(firstInputImage, secondInputImage);
  check_image_size_equal(firstInputImage, outputImage);

  const float *firstInputData = firstInputImage->GetData(MEMORYDEVICE_CPU);
  const float *secondInputData = secondInputImage->GetData(MEMORYDEVICE_CPU);
  float *outputData = outputImage->device<float>(); // note that using host<float>() doesn't work!

  const int height = outputImage->dims(0);
  const int width = outputImage->dims(1);
  const int pixelCount = height * width;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int rowMajorIndex = 0; rowMajorIndex < pixelCount; ++rowMajorIndex)
  {
    calculate_pixel_depth_difference(rowMajorIndex, firstInputData, secondInputData, width, height, outputData);
  }
}

void ImageProcessor_CPU::copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMUCharImage_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  const unsigned char *inputData = inputImage->device<unsigned char>();
  unsigned char *outputData = outputImage->GetData(MEMORYDEVICE_CPU);

  const int height = inputImage->dims(0);
  const int width = inputImage->dims(1);
  const int pixelCount = height * width;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int columnMajorIndex = 0; columnMajorIndex < pixelCount; ++columnMajorIndex)
  {
    copy_af_pixel_to_itm(columnMajorIndex, inputData, width, height, outputData);
  }
}

void ImageProcessor_CPU::copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMUChar4Image_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  af::array inputChannels[4];
  const unsigned char *inputData[4];
  for(int i = 0; i < 4; ++i)
  {
    inputChannels[i] = (*inputImage)(af::span, af::span, i);
    inputData[i] = inputChannels[i].device<unsigned char>();
  }

  Vector4u *outputData = outputImage->GetData(MEMORYDEVICE_CPU);

  const int height = outputImage->noDims.y;
  const int width = outputImage->noDims.x;
  const int pixelCount = height * width;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int columnMajorIndex = 0; columnMajorIndex < pixelCount; ++columnMajorIndex)
  {
    copy_af_pixel_to_itm(columnMajorIndex, inputData[0], inputData[1], inputData[2], inputData[3], width, height, outputData);
  }
}

void ImageProcessor_CPU::copy_itm_to_af(const ITMUChar4Image_CPtr& inputImage, const AFArray_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  const Vector4u *inputData = inputImage->GetData(MEMORYDEVICE_CPU);

  af::array outputChannels[4];
  unsigned char *outputData[4];
  for(int i = 0; i < 4; ++i)
  {
    outputChannels[i] = (*outputImage)(af::span, af::span, i);
    outputData[i] = outputChannels[i].device<unsigned char>();
  }

  const int height = inputImage->noDims.y;
  const int width = inputImage->noDims.x;
  const int pixelCount = height * width;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int rowMajorIndex = 0; rowMajorIndex < pixelCount; ++rowMajorIndex)
  {
    copy_itm_pixel_to_af(rowMajorIndex, inputData, width, height, outputData[0], outputData[1], outputData[2], outputData[3]);
  }
}

void ImageProcessor_CPU::set_on_threshold(const ITMFloatImage_CPtr& inputImage, ComparisonOperator op, float threshold, float value, const ITMFloatImage_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  const float *inputData = inputImage->GetData(MEMORYDEVICE_CPU);
  float *outputData = outputImage->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = inputImage->noDims.x * inputImage->noDims.y;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex)
  {
    set_pixel_on_threshold(pixelIndex, inputData, op, threshold, value, outputData);
  }
}

}
