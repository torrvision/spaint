/**
 * spaint: VoxelMarker_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "markers/cuda/VoxelMarker_CUDA.h"

#include "markers/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_clear_labels(SpaintVoxel *voxels, int voxelCount)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) voxels[tid].packedLabel = SpaintVoxel::PackedLabel();
}

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, SpaintVoxel::PackedLabel label, int voxelCount, SpaintVoxel::PackedLabel *oldVoxelLabels,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex, bool forceMarking)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], label, oldVoxelLabels ? &oldVoxelLabels[tid] : NULL, voxelData, voxelIndex, forceMarking);
}

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, const SpaintVoxel::PackedLabel *voxelLabels, int voxelCount, SpaintVoxel::PackedLabel *oldVoxelLabels,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex, bool forceMarking)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], voxelLabels[tid], oldVoxelLabels ? &oldVoxelLabels[tid] : NULL, voxelData, voxelIndex, forceMarking);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CUDA::clear_labels(SpaintVoxel *voxels, int voxelCount) const
{
  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_clear_labels<<<numBlocks,threadsPerBlock>>>(voxels, voxelCount);
}

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, SpaintVoxel::PackedLabel label,
                                   ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                   ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB,
                                   bool forceMarking) const
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
    forceMarking
  );
}

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                   const ORUtils::MemoryBlock<SpaintVoxel::PackedLabel>& voxelLabelsMB,
                                   ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                   ORUtils::MemoryBlock<SpaintVoxel::PackedLabel> *oldVoxelLabelsMB,
                                   bool forceMarking) const
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
    forceMarking
  );
}

}
