/**
 * spaint: ImageProcessor_CUDA.cu
 */

#include "imageprocessing/cuda/ImageProcessor_CUDA.h"

#include "imageprocessing/shared/ImageProcessor_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_absolute_difference_calculator(float *outputImage, float *firstInputImage, float *secondInputImage, Vector2i imgSize)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  shade_pixel_absolute_difference(&outputImage[locId], firstInputImage[locId], secondInputImage[locId]);
}

__global__ void ck_absolute_difference_calculator_cmrmrm(float *outputImage, float *firstInputImage, float *secondInputImage, Vector2i imgSize)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locIdcm = x * imgSize.y + y;
  int locIdrm = y * imgSize.x + x;
  shade_pixel_absolute_difference(&outputImage[locIdcm], firstInputImage[locIdrm], secondInputImage[locIdrm]);
}

__global__ void ck_pixel_setter(float *output, float *input, Vector2i imgSize, float comparator, ImageProcessor::ComparisonOperator comparisonOperator, float value)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  shade_pixel_on_comparison(&output[locId], input[locId], comparator, comparisonOperator, value);
}

__global__ void ck_set_invalid_regions(float *image, Vector2i imgSize, float value)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  shade_invalid_pixels(&image[locId], value);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessor_CUDA::absolute_difference_calculator(ITMFloatImage *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const
{
  ImageProcessor::check_image_size_equal(outputImage, firstInputImage);
  ImageProcessor::check_image_size_equal(secondInputImage, firstInputImage);

  Vector2i imgSize = outputImage->noDims;

  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  ck_absolute_difference_calculator<<<gridSize,cudaBlockSize>>>(
    outputImage->GetData(MEMORYDEVICE_CUDA),
    firstInputImage->GetData(MEMORYDEVICE_CUDA),
    secondInputImage->GetData(MEMORYDEVICE_CUDA),
    imgSize
  );
}

void ImageProcessor_CUDA::absolute_difference_calculator(af::array *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const
{
  ImageProcessor::check_image_size_equal(outputImage, firstInputImage);
  ImageProcessor::check_image_size_equal(secondInputImage, firstInputImage);

  Vector2i imgSize;
  imgSize.y = outputImage->dims(0);
  imgSize.x = outputImage->dims(1);

  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  ck_absolute_difference_calculator_cmrmrm<<<gridSize,cudaBlockSize>>>(
    outputImage->device<float>(),
    firstInputImage->GetData(MEMORYDEVICE_CUDA),
    secondInputImage->GetData(MEMORYDEVICE_CUDA),
    imgSize
  );
}

void ImageProcessor_CUDA::pixel_setter(ITMFloatImage *output, ITMFloatImage *input, float comparator, ComparisonOperator comparisonOperator, float value) const
{
  ImageProcessor::check_image_size_equal(output, input);
  Vector2i imgSize = input->noDims;

  dim3 cudaBlockSize(8,8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  ck_pixel_setter<<<gridSize,cudaBlockSize>>>(
    output->GetData(MEMORYDEVICE_CUDA),
    input->GetData(MEMORYDEVICE_CUDA),
    imgSize,
    comparator,
    comparisonOperator,
    value
    );

}

void ImageProcessor_CUDA::set_invalid_regions(ITMFloatImage *image, float value) const
{
  Vector2i imgSize = image->noDims;

  dim3 cudaBlockSize(8,8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  ck_set_invalid_regions<<<gridSize,cudaBlockSize>>>(
    image->GetData(MEMORYDEVICE_CUDA),
    imgSize,
    value
  );
}

}
