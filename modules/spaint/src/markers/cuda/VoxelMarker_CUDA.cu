/**
 * spaint: VoxelMarker_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "markers/cuda/VoxelMarker_CUDA.h"

#include "markers/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_clear_labels(SpaintVoxel *voxels, int voxelCount, ClearingSettings settings)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) clear_label(voxels[tid], settings);
}

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, SpaintVoxel::PackedLabel label, int voxelCount, SpaintVoxel::PackedLabel *oldVoxelLabels,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex, MarkingMode mode)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], label, oldVoxelLabels ? &oldVoxelLabels[tid] : NULL, voxelData, voxelIndex, mode);
}

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, const SpaintVoxel::PackedLabel *voxelLabels, int voxelCount, SpaintVoxel::PackedLabel *oldVoxelLabels,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex, MarkingMode mode)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], voxelLabels[tid], oldVoxelLabels ? &oldVoxelLabels[tid] : NULL, voxelData, voxelIndex, mode);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CUDA::clear_labels(SpaintVoxel *voxels, int voxelCount, ClearingSettings settings) const
{
  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_clear_labels<<<numBlocks,threadsPerBlock>>>(voxels, voxelCount, settings);
}

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, SpaintVoxel::PackedLabel label,
                                   SpaintScene *scene, MarkingMode mode, ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB) const
{
  int voxelCount = static_cast<int>(voxelLocationsMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_mark_voxels<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    label,
    voxelCount,
    oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CUDA) : NULL,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    mode
  );
}

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<SpaintVoxel::PackedLabel>& voxelLabelsMB,
                                   SpaintScene *scene, MarkingMode mode, ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB) const
{
  int voxelCount = static_cast<int>(voxelLocationsMB.dataSize);

  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_mark_voxels<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLabelsMB.GetData(MEMORYDEVICE_CUDA),
    voxelCount,
    oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CUDA) : NULL,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData(),
    mode
  );
}

}
