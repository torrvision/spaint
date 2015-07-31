/**
 * spaint: VoxelMarker_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_VOXELMARKER_SHARED
#define H_SPAINT_VOXELMARKER_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "VoxelMarker_Settings.h"

namespace spaint {

/**
 * \brief Decides whether or not it is possible to overwrite the old semantic label for a voxel with a new one.
 *
 * This decision is based on both the old and new labels.
 *
 * \param oldLabel  The old semantic label for the voxel.
 * \param newLabel  The new semantic label for the voxel.
 * \return          true, if it is possible to overwrite the existing label with the new one, or false otherwise.
 */
_CPU_AND_GPU_CODE_
inline bool can_overwrite_label(SpaintVoxel::PackedLabel oldLabel, SpaintVoxel::PackedLabel newLabel)
{
  // The new label can overwrite the old label iff one of the following is true:
  return
    // (a) The old label is the original (background) label.
    (oldLabel.label == 0 && oldLabel.group == SpaintVoxel::LG_USER) ||

    // (b) The old and new labels are both non-user labels.
    (oldLabel.group != SpaintVoxel::LG_USER && newLabel.group != SpaintVoxel::LG_USER) ||

    // (c) The new label was supplied by the user.
    newLabel.group == SpaintVoxel::LG_USER;
}

/**
 * \brief Clears the label of the specified voxel as necessary depending on the settings specified.
 *
 * \param voxel     The voxel whose label may be cleared.
 * \param settings  The settings to use for the label-clearing operation.
 */
_CPU_AND_GPU_CODE_
inline void clear_label(SpaintVoxel& voxel, ClearingSettings settings)
{
  SpaintVoxel::PackedLabel& packedLabel = voxel.packedLabel;

  bool shouldClear = false;
  switch(settings.mode)
  {
    case CLEAR_EQ_GROUP:  shouldClear = packedLabel.group == settings.group; break;
    case CLEAR_EQ_LABEL:  shouldClear = packedLabel.label == settings.label; break;
    case CLEAR_NEQ_GROUP: shouldClear = packedLabel.group != settings.group; break;
    case CLEAR_NEQ_LABEL: shouldClear = packedLabel.label != settings.label; break;
    default:              shouldClear = true; break;
  }

  if(shouldClear) packedLabel = SpaintVoxel::PackedLabel();
}

/**
 * \brief Marks a voxel in the scene with a semantic label.
 *
 * \param loc         The location of the voxel.
 * \param label       The semantic label with which to mark the voxel.
 * \param oldLabel    An optional location into which to store the old semantic label of the voxel.
 * \param voxelData   The scene's voxel data.
 * \param voxelIndex  The scene's voxel index.
 * \param mode        The marking mode.
 */
_CPU_AND_GPU_CODE_
inline void mark_voxel(const Vector3s& loc, SpaintVoxel::PackedLabel label, SpaintVoxel::PackedLabel *oldLabel,
                       SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *voxelIndex,
                       MarkingMode mode = NORMAL_MARKING)
{
  bool isFound;
  int voxelAddress = findVoxel(voxelIndex, loc.toInt(), isFound);
  if(isFound)
  {
    SpaintVoxel::PackedLabel oldLabelLocal = voxelData[voxelAddress].packedLabel;
    if(oldLabel) *oldLabel = oldLabelLocal;
    if(mode == FORCED_MARKING || can_overwrite_label(oldLabelLocal, label))
    {
      voxelData[voxelAddress].packedLabel = label;
    }
  }
}

}

#endif
