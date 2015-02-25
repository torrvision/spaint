/**
 * spaint: VoxelMarker_Shared.h
 */

#ifndef H_SPAINT_VOXELMARKER_SHARED
#define H_SPAINT_VOXELMARKER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief Marks a voxel in the scene with a semantic label.
 *
 * \param loc         The location of the voxel.
 * \param label       The semantic label with which to mark the voxel.
 * \param voxelData   TODO
 * \param voxelIndex  TODO
 */
_CPU_AND_GPU_CODE_
inline void mark_voxel(const Vector3s& loc, const unsigned char& label, SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex)
{
  bool isFound;
  int voxelAddress = findVoxel(voxelIndex, loc.toInt(), isFound);
  if(isFound) voxelData[voxelAddress].label = label;
}

}

#endif
