/**
 * grove: ScoreRelocaliser_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/cuda/ScoreRelocaliser_CUDA.h"
using namespace tvgutil;

#include "relocalisation/shared/ScoreRelocaliser_Shared.h"

namespace grove {

//#################### CUDA KERNELS ####################

template <int TREE_COUNT>
__global__ void ck_score_relocaliser_get_predictions(const ScorePrediction *leafPredictions, const ORUtils::VectorX<int, TREE_COUNT> *leafIndices,
                                                     ScorePrediction *outPredictions, Vector2i imgSize, int nbMaxPredictions)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if(x >= imgSize.x || y >= imgSize.y) return;

  merge_predictions_for_keypoint(x, y, leafPredictions, leafIndices, outPredictions, imgSize, nbMaxPredictions);
}

//#################### CONSTRUCTORS ####################

ScoreRelocaliser_CUDA::ScoreRelocaliser_CUDA(const std::string& forestFilename, const SettingsContainer_CPtr& settings)
: ScoreRelocaliser(forestFilename, settings, DEVICE_CUDA)
{}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreRelocaliser_CUDA::merge_predictions_for_keypoints(const LeafIndicesImage_CPtr& leafIndices, const ScorePredictionsMemoryBlock_CPtr& leafPredictions,
                                                            ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = leafIndices->noDims;
  const LeafIndices *leafIndicesData = leafIndices->GetData(MEMORYDEVICE_CUDA);

  // Leaf predictions
  const ScorePrediction *leafPredictionsData = leafPredictions->GetData(MEMORYDEVICE_CUDA);

  // NOP after the first time.
  outputPredictions->ChangeDims(imgSize);
  ScorePrediction *outPredictionsData = outputPredictions->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x, (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_score_relocaliser_get_predictions<<<gridSize, blockSize>>>(
    leafPredictionsData, leafIndicesData, outPredictionsData, imgSize, m_maxClusterCount
  );
  ORcudaKernelCheck;
}

}
