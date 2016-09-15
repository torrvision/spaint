/**
 * spaint: ImageProcessor_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
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

template <typename AFElementType, typename ITMElementType>
__global__ void ck_copy_af_to_itm(const AFElementType *inputData, int width, int height, ITMElementType *outputData)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < width * height)
  {
    copy_af_pixel_to_itm(tid, inputData, width, height, outputData);
  }
}

template <typename ITMElementType, typename AFElementType>
__global__ void ck_copy_itm_to_af(const ITMElementType *inputData, int width, int height, AFElementType *outputData)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < width * height)
  {
    copy_itm_pixel_to_af(tid, inputData, width, height, outputData);
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

//#################### HELPER FUNCTIONS ####################

/**
 * \brief TODO
 */
template <typename AFElementType, typename ITMElementType>
static void copy_af_to_itm_helper_cuda(const boost::shared_ptr<const af::array>& inputImage, const boost::shared_ptr<ORUtils::Image<ITMElementType> >& outputImage)
{
  const int height = outputImage->noDims.y;
  const int width = outputImage->noDims.x;
  const int pixelCount = height * width;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;
  ck_copy_af_to_itm<<<numBlocks,threadsPerBlock>>>(
    inputImage->device<AFElementType>(),
    width,
    height,
    outputImage->GetData(MEMORYDEVICE_CUDA)
  );

  inputImage->unlock();
}

/**
 * \brief TODO
 */
template <typename ITMElementType, typename AFElementType>
static void copy_itm_to_af_helper_cuda(const boost::shared_ptr<const ORUtils::Image<ITMElementType> >& inputImage, const boost::shared_ptr<af::array>& outputImage)
{
  const int height = inputImage->noDims.y;
  const int width = inputImage->noDims.x;
  const int pixelCount = height * width;

  int threadsPerBlock = 256;
  int numBlocks = (pixelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_copy_itm_to_af<<<numBlocks,threadsPerBlock>>>(
    inputImage->GetData(MEMORYDEVICE_CUDA),
    width,
    height,
    outputImage->device<AFElementType>()
  );

  outputImage->unlock();
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
  copy_af_to_itm_helper_cuda<unsigned char,unsigned char>(inputImage, outputImage);
}

void ImageProcessor_CUDA::copy_af_to_itm(const AFArray_CPtr& inputImage, const ITMUChar4Image_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);
  copy_af_to_itm_helper_cuda<unsigned char,Vector4u>(inputImage, outputImage);
}

void ImageProcessor_CUDA::copy_itm_to_af(const ITMUChar4Image_CPtr& inputImage, const AFArray_Ptr& outputImage) const
{
  check_image_size_equal(inputImage, outputImage);
  copy_itm_to_af_helper_cuda<Vector4u,unsigned char>(inputImage, outputImage);
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
