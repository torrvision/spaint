/**
 * spaint: VoxelMarker_CUDA.cu
 */

#include "marking/cuda/VoxelMarker_CUDA.h"

#include "marking/shared/VoxelMarker_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_mark_voxels(const Vector3s *voxelLocations, const unsigned char *voxelLabels, int voxelCount,
                               SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < voxelCount) mark_voxel(voxelLocations[tid], voxelLabels[tid], voxelData, voxelIndex);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelMarker_CUDA::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, const ORUtils::MemoryBlock<unsigned char>& voxelLabelsMB,
                                   ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene) const
{
  int voxelCount = voxelLocationsMB.dataSize;

  int threadsPerBlock = 256;
  int numBlocks = (voxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_mark_voxels<<<numBlocks,threadsPerBlock>>>(
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    voxelLabelsMB.GetData(MEMORYDEVICE_CUDA),
    voxelCount,
    scene->localVBA.GetVoxelBlocks(),
    scene->index.getIndexData()
  );
}

}
