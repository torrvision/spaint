/**
 * spaint: ImageProcessor_CUDA.cu
 */

#include "imageprocessing/cuda/ImageProcessor_CUDA.h"

#include "imageprocessing/shared/ImageProcessor_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

/**
 * \brief Calculates the pixel-wise absolute difference between two depth images.
 *
 * \param firstInputData   The data for the first input image (in row-major format).
 * \param secondInputData  The data for the second input image (in row-major format).
 * \param width            The width of each image.
 * \param height           The height of each image.
 * \param outputData       The location in which to store the result of the calculation (in column-major format).
 */
__global__ void ck_calculate_depth_difference(const float *firstInputData, const float *secondInputData, int width, int height, float *outputData)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < width * height)
  {
    calculate_pixel_depth_difference(tid, firstInputData, secondInputData, width, height, outputData);
  }
}

__global__ void ck_copy_af_to_itm(const unsigned char *inputData, int width, int height, unsigned char *outputData)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < width * height)
  {
    copy_af_pixel_to_itm(tid, inputData, width, height, outputData);
  }
}

__global__ void ck_copy_af_to_itm(const unsigned char *inputRedData, const unsigned char *inputGreenData,
                                  const unsigned char *inputBlueData, const unsigned char *inputAlphaData,
                                  int width, int height, Vector4u *outputData)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < width * height)
  {
    copy_af_pixel_to_itm(tid, inputRedData, inputGreenData, inputBlueData, inputAlphaData, width, height, outputData);
  }
}

__global__ void ck_copy_itm_to_af(const Vector4u *inputData, int width, int height,
                                  unsigned char *outputRedData, unsigned char *outputGreenData,
                                  unsigned char *outputBlueData, unsigned char *outputAlphaData)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < width * height)
  {
    copy_itm_pixel_to_af(tid, inputData, width, height, outputRedData, outputGreenData, outputBlueData, outputAlphaData);
  }
}

__global__ void ck_set_on_threshold(const float *inputData, int pixelCount, ImageProcessor::ComparisonOperator op, float threshold, float value, float *outputData)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < pixelCount)
  {
    set_pixel_on_threshold(tid, inputData, op, threshold, value, outputData);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessor_CUDA::calculate_depth_difference(const ITMFloatImage_CPtr& firstInputImage, const ITMFloatImage_CPtr& secondInputImage, const AFArray_Ptr& outputImage) const
{
  check_image_size_equal(firstInputImage, secondInputImage);
  check_image_size_equal(firstInputImage, outputImage);

  Vector2i imgSize = image_size(outputImage);
  int pixelCount = imgSize.x * imgSize.y;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;
  ck_calculate_depth_difference<<<numBlocks,threadsPerBlock>>>(
    firstInputImage->GetData(MEMORYDEVICE_CUDA),
    secondInputImage->GetData(MEMORYDEVICE_CUDA),
    imgSize.x,
    imgSize.y,
    outputImage->device<float>()
  );
}

void ImageProcessor_CUDA::copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMUCharImage_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);
  
  Vector2i imgSize = outputImage->noDims;
  int pixelCount = imgSize.x * imgSize.y;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;
  
  ck_copy_af_to_itm<<<numBlocks,threadsPerBlock>>>(
    inputImage->device<unsigned char>(),
    imgSize.x,
    imgSize.y,
    outputImage->GetData(MEMORYDEVICE_CUDA)
  );
}

void ImageProcessor_CUDA::copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMUChar4Image_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  af::array inputChannels[4];
  for(int i = 0; i < 4; ++i) inputChannels[i] = (*inputImage)(af::span, af::span, i);

  const int height = outputImage->noDims.y;
  const int width = outputImage->noDims.x;
  const int pixelCount = height * width;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_copy_af_to_itm<<<numBlocks,threadsPerBlock>>>(
    inputChannels[0].device<unsigned char>(),
    inputChannels[1].device<unsigned char>(),
    inputChannels[2].device<unsigned char>(),
    inputChannels[3].device<unsigned char>(),
    width,
    height,
    outputImage->GetData(MEMORYDEVICE_CUDA)
  );
}

void ImageProcessor_CUDA::copy_itm_to_af(const ITMUChar4Image_CPtr& inputImage, const AFArray_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  af::array outputChannels[4];
  for(int i = 0; i < 4; ++i) outputChannels[i] = (*outputImage)(af::span, af::span, i);

  const int height = inputImage->noDims.y;
  const int width = inputImage->noDims.x;
  const int pixelCount = height * width;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_copy_itm_to_af<<<numBlocks,threadsPerBlock>>>(
    inputImage->GetData(MEMORYDEVICE_CUDA),
    width,
    height,
    outputChannels[0].device<unsigned char>(),
    outputChannels[1].device<unsigned char>(),
    outputChannels[2].device<unsigned char>(),
    outputChannels[3].device<unsigned char>()
  );
}

void ImageProcessor_CUDA::set_on_threshold(const ITMFloatImage_CPtr& inputImage, ComparisonOperator op, float threshold, float value, const ITMFloatImage_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  Vector2i imgSize = inputImage->noDims;
  int pixelCount = imgSize.x * imgSize.y;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;
  ck_set_on_threshold<<<numBlocks,threadsPerBlock>>>(
    inputImage->GetData(MEMORYDEVICE_CUDA),
    pixelCount,
    op,
    threshold,
    value,
    outputImage->GetData(MEMORYDEVICE_CUDA)
  );
}

}
