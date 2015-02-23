/**
 * spaint: VoxelToCubeSelectionTransformer.cpp
 */

#include "selection/transformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"

#include "selection/transformers/shared/VoxelToCubeSelectionTransformer_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelToCubeSelectionTransformer_CPU::VoxelToCubeSelectionTransformer_CPU(int radius)
: VoxelToCubeSelectionTransformer(radius)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int VoxelToCubeSelectionTransformer_CPU::compute_output_selection_size(const ORUtils::MemoryBlock<Vector3s>& inputSelectionMB) const
{
  int cubeSideLength = 2 * m_radius + 1;
  int cubeSize = cubeSideLength * cubeSideLength * cubeSideLength;
  return inputSelectionMB.dataSize * cubeSize;
}

void VoxelToCubeSelectionTransformer_CPU::transform_selection(const ORUtils::MemoryBlock<Vector3s>& inputSelectionMB, ORUtils::MemoryBlock<Vector3s>& outputSelectionMB) const
{
  int inputVoxelCount = inputSelectionMB.dataSize;
  int outputVoxelCount = outputSelectionMB.dataSize;

  const Vector3s *inputSelection = inputSelectionMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *outputSelection = outputSelectionMB.GetData(MEMORYDEVICE_CPU);

  int cubeSideLength = 2 * m_radius + 1;
  int cubeSize = cubeSideLength * cubeSideLength * cubeSideLength;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int outputVoxelIndex = 0; outputVoxelIndex < outputVoxelCount; ++outputVoxelIndex)
  {
    write_output_voxel(outputVoxelIndex, cubeSideLength, cubeSize, m_radius, inputSelection, outputSelection);
  }
}

}
