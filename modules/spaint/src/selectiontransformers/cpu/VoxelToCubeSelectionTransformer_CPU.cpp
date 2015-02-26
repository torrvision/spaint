/**
 * spaint: VoxelToCubeSelectionTransformer_CPU.cpp
 */

#include "selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"

#include "selectiontransformers/shared/VoxelToCubeSelectionTransformer_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelToCubeSelectionTransformer_CPU::VoxelToCubeSelectionTransformer_CPU(int radius)
: VoxelToCubeSelectionTransformer(radius)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelToCubeSelectionTransformer_CPU::transform_selection(const ORUtils::MemoryBlock<Vector3s>& inputSelectionMB,
                                                              ORUtils::MemoryBlock<Vector3s>& outputSelectionMB) const
{
  const Vector3s *inputSelection = inputSelectionMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *outputSelection = outputSelectionMB.GetData(MEMORYDEVICE_CPU);
  int outputVoxelCount = outputSelectionMB.dataSize;

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int outputVoxelIndex = 0; outputVoxelIndex < outputVoxelCount; ++outputVoxelIndex)
  {
    write_output_voxel(outputVoxelIndex, m_cubeSideLength, m_cubeSize, m_radius, inputSelection, outputSelection);
  }
}

}
