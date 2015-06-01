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
 * \param firstInputImage   The first input image (in row-major format).
 * \param secondInputImage  The second input image (in row-major format).
 * \param imgSize           The size of the images.
 * \param outputImage       The image in which to store the result of the calculation (in column-major format).
 */
__global__ void ck_calculate_depth_difference(const float *firstInputImage, const float *secondInputImage, Vector2i imgSize, float *outputImage)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int rowMajorIndex = y * imgSize.x + x;
  int columnMajorIndex = x * imgSize.y + y;
  calculate_pixel_depth_difference(firstInputImage[rowMajorIndex], secondInputImage[rowMajorIndex], &outputImage[columnMajorIndex]);
}

__global__ void ck_set_on_threshold(const float *input, Vector2i imgSize, ImageProcessor::ComparisonOperator op, float threshold, float value, float *output)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  set_pixel_on_threshold(input[locId], op, threshold, value, &output[locId]);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessor_CUDA::calculate_depth_difference(const ITMFloatImage_CPtr& firstInputImage, const ITMFloatImage_CPtr& secondInputImage, const AFImage_Ptr& outputImage) const
{
  check_image_size_equal(firstInputImage, secondInputImage);
  check_image_size_equal(firstInputImage, outputImage);

  Vector2i imgSize = image_size(outputImage);

  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  ck_calculate_depth_difference<<<gridSize,cudaBlockSize>>>(
    firstInputImage->GetData(MEMORYDEVICE_CUDA),
    secondInputImage->GetData(MEMORYDEVICE_CUDA),
    imgSize,
    outputImage->device<float>()
  );
}

void ImageProcessor_CUDA::set_on_threshold(const ITMFloatImage_CPtr& inputImage, ComparisonOperator op, float threshold, float value, const ITMFloatImage_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);

  Vector2i imgSize = inputImage->noDims;

  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  ck_set_on_threshold<<<gridSize,cudaBlockSize>>>(
    inputImage->GetData(MEMORYDEVICE_CUDA),
    imgSize,
    op,
    threshold,
    value,
    outputImage->GetData(MEMORYDEVICE_CUDA)
  );

}

}
