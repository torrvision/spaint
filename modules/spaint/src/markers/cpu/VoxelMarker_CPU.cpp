/**
 * spaint: VoxelMarker_CPU.cpp
 */

#include "markers/cpu/VoxelMarker_CPU.h"

#include "markers/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CPU::clear_labels(SpaintVoxel *voxels, int voxelCount) const
{
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < voxelCount; ++i)
  {
    voxels[i].label = 0;
  }
}

void VoxelMarker_CPU::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, SpaintVoxel::LabelType label,
                                  ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                  ORUtils::MemoryBlock<SpaintVoxel::LabelType> *oldVoxelLabelsMB) const
{
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  unsigned char *oldVoxelLabels = oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CPU) : NULL;
  int voxelCount = voxelLocationsMB.dataSize;

  SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < voxelCount; ++i)
  {
    mark_voxel(voxelLocations[i], label, oldVoxelLabels ? &oldVoxelLabels[i] : NULL, voxelData, voxelIndex);
  }
}

void VoxelMarker_CPU::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                  const ORUtils::MemoryBlock<SpaintVoxel::LabelType>& voxelLabelsMB,
                                  ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                  ORUtils::MemoryBlock<SpaintVoxel::LabelType> *oldVoxelLabelsMB) const
{
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  const unsigned char *voxelLabels = voxelLabelsMB.GetData(MEMORYDEVICE_CPU);
  unsigned char *oldVoxelLabels = oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CPU) : NULL;
  int voxelCount = voxelLocationsMB.dataSize;

  SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < voxelCount; ++i)
  {
    mark_voxel(voxelLocations[i], voxelLabels[i], oldVoxelLabels ? &oldVoxelLabels[i] : NULL, voxelData, voxelIndex);
  }
}

}
