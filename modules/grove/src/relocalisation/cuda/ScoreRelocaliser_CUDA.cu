/**
 * grove: ScoreRelocaliser_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/cuda/ScoreRelocaliser_CUDA.h"
using namespace tvgutil;

#include <thrust/count.h>
#include <thrust/device_ptr.h>

#include "relocalisation/shared/ScoreRelocaliser_Shared.h"

namespace grove {

//#################### ANONYMOUS STRUCTS ####################
namespace
{

struct ValidDepth
{
  _CPU_AND_GPU_CODE_ bool operator()(const float &x)
  {
    return x > 0.0f;
  }
};

}

//#################### CUDA KERNELS ####################

template <int TREE_COUNT>
__global__ void ck_merge_predictions_for_keypoints(const ORUtils::VectorX<int,TREE_COUNT> *leafIndices, const ScorePrediction *predictionsBlock,
                                                   Vector2i imgSize, int maxClusterCount, ScorePrediction *outputPredictions)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if(x < imgSize.x && y < imgSize.y)
  {
    merge_predictions_for_keypoint(x, y, leafIndices, predictionsBlock, imgSize, maxClusterCount, outputPredictions);
  }
}

//#################### CONSTRUCTORS ####################

ScoreRelocaliser_CUDA::ScoreRelocaliser_CUDA(const std::string& forestFilename, const SettingsContainer_CPtr& settings)
: ScoreRelocaliser(forestFilename, settings, DEVICE_CUDA)
{}

//#################### PROTECTED MEMBER FUNCTIONS ####################

uint32_t ScoreRelocaliser_CUDA::count_valid_depths(const ORFloatImage *depthImage) const
{
  uint32_t validDepths = 0;

  const int imgArea = depthImage->noDims.width * depthImage->noDims.height;
  const float *depths = depthImage->GetData(MEMORYDEVICE_CUDA);

  thrust::device_ptr<const float> depthsStart(depths);
  thrust::device_ptr<const float> depthsEnd(depths + imgArea);

  // Do the counting.
  return validDepths = thrust::count_if(depthsStart, depthsEnd, ValidDepth());
}

void ScoreRelocaliser_CUDA::merge_predictions_for_keypoints(const LeafIndicesImage_CPtr& leafIndices, ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = leafIndices->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  const LeafIndices *leafIndicesPtr = leafIndices->GetData(MEMORYDEVICE_CUDA);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CUDA);
  const ScorePrediction *predictionsBlockPtr = m_relocaliserState->predictionsBlock->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x, (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_merge_predictions_for_keypoints<<<gridSize, blockSize>>>(
    leafIndicesPtr, predictionsBlockPtr, imgSize, m_maxClusterCount, outputPredictionsPtr
  );
  ORcudaKernelCheck;
}

}
