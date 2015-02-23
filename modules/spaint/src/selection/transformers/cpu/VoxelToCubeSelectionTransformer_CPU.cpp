/**
 * spaint: VoxelToCubeSelectionTransformer.cpp
 */

#include "selection/transformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"

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
  for(int i = 0; i < outputVoxelCount; ++i)
  {
    int inputVoxelIndex = i / cubeSize;
    const Vector3s& inputVoxel = inputSelection[inputVoxelIndex];

    int xOffset = i % cubeSideLength - m_radius;
    int yOffset = i % (cubeSideLength * cubeSideLength) / cubeSideLength - m_radius;
    int zOffset = i / (cubeSideLength * cubeSideLength) - m_radius;

    outputSelection[i] = Vector3s(inputVoxel.x + xOffset, inputVoxel.y + yOffset, inputVoxel.z + zOffset);
  }
}

}
