/**
 * spaint: LabelPropagator_CPU.cpp
 */

#include "propagation/cpu/LabelPropagator_CPU.h"

#include "propagation/shared/LabelPropagator_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LabelPropagator_CPU::LabelPropagator_CPU(size_t raycastResultSize)
: LabelPropagator(raycastResultSize)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void LabelPropagator_CPU::calculate_normals(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  const Vector4f *raycastResultData = raycastResult->GetData(MEMORYDEVICE_CPU);
  const size_t raycastResultSize = raycastResult->dataSize;
  Vector3f *surfaceNormals = m_surfaceNormalsMB->GetData(MEMORYDEVICE_CPU);
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < raycastResultSize; ++voxelIndex)
  {
    write_surface_normal(voxelIndex, raycastResultData, voxelData, indexData, surfaceNormals);
  }
}

}
