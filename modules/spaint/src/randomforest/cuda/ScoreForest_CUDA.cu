/**
 * spaint: ScoreForest_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/ScoreForest_CUDA.h"

#include "randomforest/shared/ScoreForest_Shared.h"

namespace spaint
{
__global__ void ck_evaluate_forest(const ScoreForest::NodeEntry* forestTexture,
    const RGBDPatchDescriptor* descriptorsData, Vector2i imgSize,
    ScoreForest::LeafIndices* leafData)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  evaluate_forest_shared(forestTexture, descriptorsData, imgSize, leafData, x,
      y);
}

__global__ void ck_get_predictions(const Prediction3DColour* leafPredictions,
    const ScoreForest::LeafIndices* leafIndices,
    Prediction3DColour* outPredictions, Vector2i imgSize)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  get_prediction_for_leaf_shared(leafPredictions, leafIndices, outPredictions,
      imgSize, x, y);
}

ScoreForest_CUDA::ScoreForest_CUDA(const std::string &fileName) :
    ScoreForest(fileName)
{
}

void ScoreForest_CUDA::find_leaves(
    const RGBDPatchDescriptorImage_CPtr &descriptors,
    LeafIndicesImage_Ptr &leaf_indices) const
{
  const NodeEntry* forestTexture = m_nodeImage->GetData(MEMORYDEVICE_CUDA);

  const Vector2i imgSize = descriptors->noDims;
  const RGBDPatchDescriptor* descriptorsData = descriptors->GetData(
      MEMORYDEVICE_CUDA);

  leaf_indices->ChangeDims(imgSize);
  LeafIndices* leafData = leaf_indices->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x,
      (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_evaluate_forest<<<gridSize,blockSize>>>(forestTexture, descriptorsData, imgSize, leafData);
  ORcudaKernelCheck;
}

void ScoreForest_CUDA::get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
    ScorePredictionsImage_Ptr &predictions) const
{
  const Vector2i imgSize = leaf_indices->noDims;
  const LeafIndices* leafIndices = leaf_indices->GetData(MEMORYDEVICE_CUDA);

// Leaf predictions
  const Prediction3DColour *leafPredictionsData = m_predictionsBlock->GetData(
      MEMORYDEVICE_CUDA);

// ~12MB for 160x120 image
  predictions->ChangeDims(imgSize);
  Prediction3DColour *outPredictionsData = predictions->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x,
      (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_get_predictions<<<gridSize,blockSize>>>(leafPredictionsData, leafIndices, outPredictionsData,
      imgSize);
  ORcudaKernelCheck;
}

Prediction3DColour ScoreForest_CUDA::get_prediction(size_t treeIdx,
    size_t leafIdx) const
{
  if (treeIdx >= get_nb_trees())
    throw std::runtime_error("invalid treeIdx");
  if (leafIdx >= get_nb_leaves_in_tree(treeIdx))
    throw std::runtime_error("invalid leafIdx");

  size_t linearizedLeafIdx = leafIdx;
  for (int i = 0; i < treeIdx; ++i)
    linearizedLeafIdx += get_nb_leaves_in_tree(i);

  return m_predictionsBlock->GetElement(linearizedLeafIdx, MEMORYDEVICE_CUDA);
}

//#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS

ScoreForest_CUDA::ScoreForest_CUDA(const EnsembleLearner &pretrained_forest) :
    ScoreForest(pretrained_forest)
{
}

#endif
}
