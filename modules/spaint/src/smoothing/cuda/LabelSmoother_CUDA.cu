/**
 * spaint: LabelSmoother_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "smoothing/cuda/LabelSmoother_CUDA.h"

#include "smoothing/shared/LabelSmoother_Shared.h"

#define DEBUGGING 0

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_smooth_from_neighbours(const Vector4f *raycastResultData, int raycastResultSize, int width, int height, int maxLabelCount,
                                          SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData, float maxSquaredDistanceBetweenVoxels)
{
  int voxelIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelIndex < raycastResultSize)
  {
    smooth_from_neighbours(voxelIndex, width, height, maxLabelCount, raycastResultData, voxelData, indexData, maxSquaredDistanceBetweenVoxels);
  }
}

//#################### CONSTRUCTORS ####################

LabelSmoother_CUDA::LabelSmoother_CUDA(size_t maxLabelCount, float maxSquaredDistanceBetweenVoxels)
: LabelSmoother(maxLabelCount, maxSquaredDistanceBetweenVoxels)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LabelSmoother_CUDA::smooth_labels(const ORFloat4Image *raycastResult, SpaintVoxelScene *scene) const
{
  const int raycastResultSize = static_cast<int>(raycastResult->dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (raycastResultSize + threadsPerBlock - 1) / threadsPerBlock;

  ck_smooth_from_neighbours<<<numBlocks,threadsPerBlock>>>(
    raycastResult->GetData(MEMORYDEVICE_CUDA),
    raycastResultSize,
    raycastResult->noDims.x,
    raycastResult->noDims.y,
    static_cast<int>(m_maxLabelCount),
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    m_maxSquaredDistanceBetweenVoxels
  );
}

}
