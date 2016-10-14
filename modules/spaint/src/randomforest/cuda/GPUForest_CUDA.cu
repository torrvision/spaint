/**
 * spaint: GPUForest_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cuda/GPUForest_CUDA.h"

#include "randomforest/shared/GPUForest_Shared.h"

namespace spaint
{
__global__ void ck_evaluate_forest(const GPUForestNode* forestTexture,
    int nbTrees, const RGBDPatchFeature* featureData, Vector2i imgSize,
    int* leafData, Vector2i leafSize)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  evaluate_forest(forestTexture, nbTrees, featureData, imgSize, leafData, leafSize, x, y);
}

GPUForest_CUDA::GPUForest_CUDA(const EnsembleLearner &pretrained_forest) :
    GPUForest(pretrained_forest)
{
  m_forestImage->UpdateDeviceFromHost();
}

void GPUForest_CUDA::evaluate_forest(const RGBDPatchFeatureImage_CPtr &features,
    ITMIntImage_Ptr &leaf_indices) const
{
  const int nbTrees = m_forestImage->noDims.width;
  const GPUForestNode* forestTexture = m_forestImage->GetData(
      MEMORYDEVICE_CUDA);

  const Vector2i imgSize = features->noDims;
  const RGBDPatchFeature* featureData = features->GetData(MEMORYDEVICE_CUDA);

  const Vector2i leafSize(imgSize.x * imgSize.y, nbTrees);
  leaf_indices->ChangeDims(leafSize);
  int* leafData = leaf_indices->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x,
      (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_evaluate_forest<<<gridSize,blockSize>>>(forestTexture, nbTrees, featureData, imgSize, leafData, leafSize);
  cudaDeviceSynchronize();
}
}
