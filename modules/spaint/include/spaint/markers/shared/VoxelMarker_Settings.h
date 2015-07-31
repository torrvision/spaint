/**
 * spaint: VoxelMarker_Settings.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_VOXELMARKER_SETTINGS
#define H_SPAINT_VOXELMARKER_SETTINGS

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief The values of this enumeration represent the different label-clearing modes that are supported.
 */
enum ClearingMode
{
  /** Clear all voxels regardless of their current labels. */
  CLEAR_ALL,

  /** Only clear voxels that have a specific label. */
  CLEAR_EQ_LABEL,

  /** Only clear voxels that have a specific label and are not in a specific group. */
  CLEAR_EQ_LABEL_NEQ_GROUP,

  /** Only clear voxels whose labels are not in a specific group. */
  CLEAR_NEQ_GROUP
};

/**
 * \brief An instance of this struct can be used to specify the settings for a label-clearing operation.
 */
struct ClearingSettings
{
  //#################### PUBLIC VARIABLES ####################

  /** The group to clear or preserve (only relevant in group-based modes). */
  const unsigned char group;

  /** The label to clear or preserve (only relevant in label-based modes). */
  const unsigned char label;

  /** The label-clearing mode. */
  const ClearingMode mode;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs a set of label-clearing settings.
   *
   * \param mode_   The label-clearing mode.
   * \param group_  The group to clear or preserve (only relevant in group-based modes).
   * \param label_  The label to clear or preserve (only relevant in label-based modes).
   */
  ClearingSettings(ClearingMode mode_, unsigned char group_, unsigned char label_)
  : group(group_), label(label_), mode(mode_)
  {}
};

/**
 * \brief The values of this enumeration represent the different marking modes that are supported.
 */
enum MarkingMode
{
  /** In forced marking mode, an old label is always overwritten with a new one without a test being performed. */
  FORCED_MARKING,

  /** In normal marking mode, an old label is only overwritten with a new one if the two labels pass a test. */
  NORMAL_MARKING
};

}

#endif
