/**
 * spaint: ImageProcessing_CUDA.cu
 */

#include "imageprocessing/cuda/ImageProcessing_CUDA.h"

#include "imageprocessing/shared/ImageProcessing_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_absolute_difference_calculator(float *outputImage, float *firstInputImage, float *secondInputImage, Vector2i imgSize)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y + blockDim.y + threadIdx.y;
  if(x >= imgSize.x || y >= imgSize.y) return;

  int locId = y * imgSize.x + x;
  shade_pixel_absolute_difference(&outputImage[locId], firstInputImage[locId], secondInputImage[locId]);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ImageProcessing_CUDA::absolute_difference_calculator(ITMFloatImage *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const
{
  ImageProcessing::check_image_size_equal(outputImage, firstInputImage);
  ImageProcessing::check_image_size_equal(secondInputImage, firstInputImage);

  Vector2i imgSize = outputImage->noDims;

  dim3 cudaBlockSize(8, 8);
  dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
  printf("gridSize = %d x %d\n", gridSize.x, gridSize.y);
  ck_absolute_difference_calculator<<<gridSize,cudaBlockSize>>>(
    outputImage->GetData(MEMORYDEVICE_CUDA),
    firstInputImage->GetData(MEMORYDEVICE_CUDA),
    secondInputImage->GetData(MEMORYDEVICE_CUDA),
    imgSize
  );
}

}
