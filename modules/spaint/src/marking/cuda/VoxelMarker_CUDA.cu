/**
 * spaint: VoxelMarker_CUDA.cu
 */

#include "marking/cuda/VoxelMarker_CUDA.h"

#include "marking/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, unsigned char label, int voxelCount,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], label, voxelData, voxelIndex);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label, ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  int voxelCount = voxelLocationsMB.dataSize;

  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_mark_voxels<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    label,
    voxelCount,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData()
  );
}

}
