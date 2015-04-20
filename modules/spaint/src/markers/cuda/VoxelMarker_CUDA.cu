/**
 * spaint: VoxelMarker_CUDA.cu
 */

#include "markers/cuda/VoxelMarker_CUDA.h"

#include "markers/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_clear_labels(SpaintVoxel *voxels, int voxelCount)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) voxels[tid].label = 0;
}

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, SpaintVoxel::LabelType label, int voxelCount, SpaintVoxel::LabelType *oldVoxelLabels,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], label, oldVoxelLabels ? &oldVoxelLabels[tid] : NULL, voxelData, voxelIndex);
}

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, const SpaintVoxel::LabelType *voxelLabels, int voxelCount, SpaintVoxel::LabelType *oldVoxelLabels,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], voxelLabels[tid], oldVoxelLabels ? &oldVoxelLabels[tid] : NULL, voxelData, voxelIndex);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CUDA::clear_labels(SpaintVoxel *voxels, int voxelCount) const
{
  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_clear_labels<<<numBlocks,threadsPerBlock>>>(voxels, voxelCount);
}

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, SpaintVoxel::LabelType label,
                                   ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                   ORUtils::MemoryBlock<SpaintVoxel::LabelType> *oldVoxelLabelsMB) const
{
  int voxelCount = voxelLocationsMB.dataSize;

  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_mark_voxels<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    label,
    voxelCount,
    oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CUDA) : NULL,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData()
  );
}

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB,
                                   const ORUtils::MemoryBlock<SpaintVoxel::LabelType>& voxelLabelsMB,
                                   ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                   ORUtils::MemoryBlock<SpaintVoxel::LabelType> *oldVoxelLabelsMB) const
{
  int voxelCount = voxelLocationsMB.dataSize;

  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_mark_voxels<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLabelsMB.GetData(MEMORYDEVICE_CUDA),
    voxelCount,
    oldVoxelLabelsMB ? oldVoxelLabelsMB->GetData(MEMORYDEVICE_CUDA) : NULL,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData()
  );
}

}
