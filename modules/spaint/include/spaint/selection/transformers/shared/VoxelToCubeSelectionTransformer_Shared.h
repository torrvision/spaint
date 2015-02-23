/**
 * spaint: VoxelToCubeSelectionTransformer_Shared.h
 */

#ifndef H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER_SHARED
#define H_SPAINT_VOXELTOCUBESELECTIONTRANSFORMER_SHARED

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void write_output_voxel(int outputVoxelIndex, int cubeSideLength, int cubeSize, int radius, const Vector3s *inputSelection, Vector3s *outputSelection)
{
  int inputVoxelIndex = outputVoxelIndex / cubeSize;
  const Vector3s& inputVoxel = inputSelection[inputVoxelIndex];

  int xOffset = outputVoxelIndex % cubeSideLength - radius;
  int yOffset = outputVoxelIndex % (cubeSideLength * cubeSideLength) / cubeSideLength - radius;
  int zOffset = outputVoxelIndex / (cubeSideLength * cubeSideLength) - radius;

  outputSelection[outputVoxelIndex] = Vector3s(inputVoxel.x + xOffset, inputVoxel.y + yOffset, inputVoxel.z + zOffset);
}

}

#endif
