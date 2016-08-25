/**
 * spaint: VoxelMarker_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "markers/cpu/VoxelMarker_CPU.h"

#include "markers/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CPU::clear_labels(SpaintVoxel *voxels, int voxelCount, ClearingSettings settings) const
{
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < voxelCount; ++i)
  {
    clear_label(voxels[i], settings);
  }
}

void VoxelMarker_CPU::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, SpaintVoxel::PackedLabel label,
                                  SpaintVoxelScene *scene, MarkingMode mode, ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB) const
{
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  SpaintVoxel::PackedLabel *oldVoxelLabels = oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CPU) : NULL;
  int voxelCount = static_cast<int>(voxelLocationsMB.dataSize);

  SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < voxelCount; ++i)
  {
    mark_voxel(voxelLocations[i], label, oldVoxelLabels ? &oldVoxelLabels[i] : NULL, voxelData, voxelIndex, mode);
  }
}

void VoxelMarker_CPU::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<SpaintVoxel::PackedLabel>& voxelLabelsMB,
                                  SpaintVoxelScene *scene, MarkingMode mode, ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB) const
{
  const Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  const SpaintVoxel::PackedLabel *voxelLabels = voxelLabelsMB.GetData(MEMORYDEVICE_CPU);
  SpaintVoxel::PackedLabel *oldVoxelLabels = oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CPU) : NULL;
  int voxelCount = static_cast<int>(voxelLocationsMB.dataSize);

  SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *voxelIndex = scene->index.getIndexData();

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < voxelCount; ++i)
  {
    mark_voxel(voxelLocations[i], voxelLabels[i], oldVoxelLabels ? &oldVoxelLabels[i] : NULL, voxelData, voxelIndex, mode);
  }
}

}
