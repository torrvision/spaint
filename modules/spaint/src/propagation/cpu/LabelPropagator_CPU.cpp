/**
 * spaint: LabelPropagator_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "propagation/cpu/LabelPropagator_CPU.h"

#include "propagation/shared/LabelPropagator_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelPropagator_CPU::LabelPropagator_CPU(size_t raycastResultSize, float maxAngleBetweenNormals, float maxSquaredDistanceBetweenColours, float maxSquaredDistanceBetweenVoxels)
: LabelPropagator(raycastResultSize, maxAngleBetweenNormals, maxSquaredDistanceBetweenColours, maxSquaredDistanceBetweenVoxels)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void LabelPropagator_CPU::calculate_normals(const ITMFloat4Image *raycastResult, const SpaintVoxelScene *scene) const
{
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  const Vector4f *raycastResultData = raycastResult->GetData(MEMORYDEVICE_CPU);
  const int raycastResultSize = static_cast<int>(raycastResult->dataSize);
  Vector3f *surfaceNormals = m_surfaceNormalsMB->GetData(MEMORYDEVICE_CPU);
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < raycastResultSize; ++voxelIndex)
  {
    write_surface_normal(voxelIndex, raycastResultData, voxelData, indexData, surfaceNormals);
  }
}

void LabelPropagator_CPU::perform_propagation(SpaintVoxel::Label label, const ITMFloat4Image *raycastResult, SpaintVoxelScene *scene) const
{
  const int height = raycastResult->noDims.y;
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  const Vector4f *raycastResultData = raycastResult->GetData(MEMORYDEVICE_CPU);
  const int raycastResultSize = static_cast<int>(raycastResult->dataSize);
  const Vector3f *surfaceNormals = m_surfaceNormalsMB->GetData(MEMORYDEVICE_CPU);
  SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const int width = raycastResult->noDims.x;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < raycastResultSize; ++voxelIndex)
  {
    propagate_from_neighbours(
      voxelIndex, width, height, label, raycastResultData, surfaceNormals, voxelData, indexData,
      m_maxAngleBetweenNormals, m_maxSquaredDistanceBetweenColours, m_maxSquaredDistanceBetweenVoxels
    );
  }
}

}
