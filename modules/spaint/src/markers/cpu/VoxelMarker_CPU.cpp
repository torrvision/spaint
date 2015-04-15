/**
 * spaint: VoxelMarker_CPU.cpp
 */

#include "markers/cpu/VoxelMarker_CPU.h"

#include "markers/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CPU::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, SpaintVoxel::LabelType label,
                                  ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  int voxelCount = voxelLocationsMB.dataSize;

  SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < voxelCount; ++i)
  {
    mark_voxel(voxelLocations[i], label, voxelData, voxelIndex);
  }
}

}
