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
  /** All voxels are cleared regardless of their current labels. */
  CLEAR_ALL,

  /** Only clear voxels whose labels are in a specific group. */
  CLEAR_EQ_GROUP,

  /** Only clear voxels that have a specific label. */
  CLEAR_EQ_LABEL,

  /** Only clear voxels that have a specific label and are not in a specific group. */
  CLEAR_EQ_LABEL_NEQ_GROUP,

  /** Only clear voxels whose labels are not in a specific group. */
  CLEAR_NEQ_GROUP,

  /** Only clear voxels that do not have a specific label. */
  CLEAR_NEQ_LABEL
};

/**
 * \brief An instance of this struct can be used to specify the settings for a label-clearing operation.
 */
struct ClearingSettings
{
  //#################### PUBLIC VARIABLES ####################

  /** The label-clearing mode. */
  const ClearingMode mode;

  /** The group to clear or preserve (only relevant in the CLEAR_EQ_GROUP or CLEAR_NEQ_GROUP modes). */
  const unsigned char group;

  /** The label to clear or preserve (only relevant in the CLEAR_EQ_LABEL or CLEAR_NEQ_LABEL modes). */
  const unsigned char label;

  //#################### CONSTRUCTORS ####################

  ClearingSettings(ClearingMode mode_, unsigned char group_, unsigned char label_)
  : mode(mode_), group(group_), label(label_)
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
