/**
 * spaint: UniformVoxelSampler_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_UNIFORMVOXELSAMPLER_SHARED
#define H_SPAINT_UNIFORMVOXELSAMPLER_SHARED

#include <ITMLib/Objects/Scene/ITMRepresentationAccess.h>

namespace spaint {

/**
 * \brief Writes the location of a voxel to be sampled from the raycast result into the sampled voxel locations array.
 *
 * Each thread writes the location of a single voxel.
 *
 * \param tid                   The thread ID.
 * \param raycastResult         The current raycast result.
 * \param sampledVoxelIndices   The indices of the voxels to be sampled from the raycast result.
 * \param sampledVoxelLocations An array into which to write the locations of the sampled voxels.
 */
_CPU_AND_GPU_CODE_
inline void write_sampled_voxel_location(int tid, const Vector4f *raycastResult, const int *sampledVoxelIndices, Vector3s *sampledVoxelLocations)
{
  Vector4f loc = raycastResult[sampledVoxelIndices[tid]];
  sampledVoxelLocations[tid] = loc.w > 0 ? loc.toVector3().toShortRound() : Vector3s(0,0,0);
}

}

#endif
