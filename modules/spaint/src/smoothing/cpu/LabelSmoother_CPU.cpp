/**
 * spaint: LabelSmoother_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "smoothing/cpu/LabelSmoother_CPU.h"

#include "smoothing/shared/LabelSmoother_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelSmoother_CPU::LabelSmoother_CPU(size_t maxLabelCount, float maxSquaredDistanceBetweenVoxels)
: LabelSmoother(maxLabelCount, maxSquaredDistanceBetweenVoxels)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LabelSmoother_CPU::smooth_labels(const ORFloat4Image *raycastResult, SpaintVoxelScene *scene) const
{
  const int height = raycastResult->noDims.y;
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  const Vector4f *raycastResultData = raycastResult->GetData(MEMORYDEVICE_CPU);
  const int raycastResultSize = static_cast<int>(raycastResult->dataSize);
  SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const int width = raycastResult->noDims.x;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < raycastResultSize; ++voxelIndex)
  {
    smooth_from_neighbours(voxelIndex, width, height, static_cast<int>(m_maxLabelCount), raycastResultData, voxelData, indexData, m_maxSquaredDistanceBetweenVoxels);
  }
}

}
