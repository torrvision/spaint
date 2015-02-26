/**
 * spaint: VoxelToCubeSelectionTransformer_CUDA.cu
 */

#include "selectiontransformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h"

#include "selectiontransformers/shared/VoxelToCubeSelectionTransformer_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_transform_selection(int cubeSideLength, int cubeSize, int radius, const Vector3s *inputSelection, Vector3s *outputSelection, int outputVoxelCount)
{
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if(tid < outputVoxelCount) write_output_voxel(tid, cubeSideLength, cubeSize, radius, inputSelection, outputSelection);
}

//#################### CONSTRUCTORS ####################

VoxelToCubeSelectionTransformer_CUDA::VoxelToCubeSelectionTransformer_CUDA(int radius)
: VoxelToCubeSelectionTransformer(radius)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelToCubeSelectionTransformer_CUDA::transform_selection(const Selection& inputSelectionMB, Selection& outputSelectionMB) const
{
  int outputVoxelCount = outputSelectionMB.dataSize;

  int threadsPerBlock = 256;
  int numBlocks = (outputVoxelCount + threadsPerBlock - 1) / threadsPerBlock;

  ck_transform_selection<<<numBlocks,threadsPerBlock>>>(
    m_cubeSideLength,
    m_cubeSize,
    m_radius,
    inputSelectionMB.GetData(MEMORYDEVICE_CUDA),
    outputSelectionMB.GetData(MEMORYDEVICE_CUDA),
    outputVoxelCount
  );
}

}
