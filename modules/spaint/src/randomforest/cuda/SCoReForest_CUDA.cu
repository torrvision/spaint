/**
 * spaint: SCoReForest_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/SCoReForest_CUDA.h"

#include "randomforest/shared/SCoReForest_Shared.h"

namespace spaint
{
__global__ void ck_evaluate_forest(const SCoReForest::NodeEntry* forestTexture,
    const RGBDPatchFeature* featureData, Vector2i imgSize,
    SCoReForest::LeafIndices* leafData)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  evaluate_forest_shared(forestTexture, featureData, imgSize, leafData, x, y);
}

__global__ void ck_get_predictions(const GPUForestPrediction* leafPredictions,
    const SCoReForest::LeafIndices* leafIndices,
    GPUForestPrediction* outPredictions, Vector2i imgSize)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  get_prediction_for_leaf_shared(leafPredictions, leafIndices, outPredictions,
      imgSize, x, y);
}

SCoReForest_CUDA::SCoReForest_CUDA(const std::string &fileName) :
    SCoReForest(fileName)
{
}

void SCoReForest_CUDA::find_leaves(const RGBDPatchFeatureImage_CPtr &features,
    LeafIndicesImage_Ptr &leaf_indices) const
{
  const NodeEntry* forestTexture = m_nodeImage->GetData(MEMORYDEVICE_CUDA);

  const Vector2i imgSize = features->noDims;
  const RGBDPatchFeature* featureData = features->GetData(MEMORYDEVICE_CUDA);

  leaf_indices->ChangeDims(imgSize);
  LeafIndices* leafData = leaf_indices->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x,
      (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_evaluate_forest<<<gridSize,blockSize>>>(forestTexture, featureData, imgSize, leafData);
  ORcudaKernelCheck;
}

void SCoReForest_CUDA::get_predictions(const LeafIndicesImage_Ptr &leaf_indices,
    GPUForestPredictionsImage_Ptr &predictions) const
{
  const Vector2i imgSize = leaf_indices->noDims;
  const LeafIndices* leafIndices = leaf_indices->GetData(MEMORYDEVICE_CUDA);

// Leaf predictions
  const GPUForestPrediction *leafPredictionsData = m_predictionsBlock->GetData(
      MEMORYDEVICE_CUDA);

// ~12MB for 160x120 image
  predictions->ChangeDims(imgSize);
  GPUForestPrediction *outPredictionsData = predictions->GetData(
      MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x,
      (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_get_predictions<<<gridSize,blockSize>>>(leafPredictionsData, leafIndices, outPredictionsData,
      imgSize);
  ORcudaKernelCheck;
}

GPUForestPrediction SCoReForest_CUDA::get_prediction(size_t treeIdx,
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

SCoReForest_CUDA::SCoReForest_CUDA(const EnsembleLearner &pretrained_forest) :
    SCoReForest(pretrained_forest)
{
}

#endif
}
