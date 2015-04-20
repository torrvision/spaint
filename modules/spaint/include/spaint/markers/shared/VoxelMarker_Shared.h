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
 * \param oldLabel    An optional location into which to store the old semantic label of the voxel.
 * \param voxelData   The scene's voxel data.
 * \param voxelIndex  The scene's voxel index.
 */
_CPU_AND_GPU_CODE_
inline void mark_voxel(const Vector3s& loc, SpaintVoxel::LabelType label, SpaintVoxel::LabelType *oldLabel,
                       SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex)
{
  bool isFound;
  int voxelAddress = findVoxel(voxelIndex, loc.toInt(), isFound);
  if(isFound)
  {
    if(oldLabel) *oldLabel = voxelData[voxelAddress].label;
    voxelData[voxelAddress].label = label;
  }
}

}

#endif
