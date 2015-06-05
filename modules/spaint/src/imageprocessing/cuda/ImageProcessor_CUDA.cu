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
