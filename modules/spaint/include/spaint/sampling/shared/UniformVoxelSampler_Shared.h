/**
 * spaint: UniformVoxelSampler_Shared.h
 */

#ifndef H_SPAINT_UNIFORMVOXELSAMPLER_SHARED
#define H_SPAINT_UNIFORMVOXELSAMPLER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

namespace spaint {

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void write_sampled_voxel_location(int tid, const Vector4f *raycastResult, const int *sampledVoxelIndices, Vector3s *sampledVoxelLocations)
{
  Vector4f loc = raycastResult[sampledVoxelIndices[tid]];
  sampledVoxelLocations[tid] = loc.w > 0 ? loc.toVector3().toShortRound() : Vector3s(0,0,0);
}

}

#endif
