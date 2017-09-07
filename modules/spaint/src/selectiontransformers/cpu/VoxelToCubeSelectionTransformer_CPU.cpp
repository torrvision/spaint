/**
 * spaint: VoxelToCubeSelectionTransformer_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"
using namespace ITMLib;

#include "selectiontransformers/shared/VoxelToCubeSelectionTransformer_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelToCubeSelectionTransformer_CPU::VoxelToCubeSelectionTransformer_CPU(int radius)
: VoxelToCubeSelectionTransformer(radius, ITMLibSettings::DEVICE_CPU)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelToCubeSelectionTransformer_CPU::transform_selection(const Selection& inputSelectionMB, Selection& outputSelectionMB) const
{
  const int cubeSideLength = cube_side_length();
  const int cubeSize = cube_size();
  const Vector3s *inputSelection = inputSelectionMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *outputSelection = outputSelectionMB.GetData(MEMORYDEVICE_CPU);
  int outputVoxelCount = static_cast<int>(outputSelectionMB.dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int outputVoxelIndex = 0; outputVoxelIndex < outputVoxelCount; ++outputVoxelIndex)
  {
    write_voxel_to_output_selection(outputVoxelIndex, cubeSideLength, cubeSize, m_radius, inputSelection, outputSelection);
  }
}

}
