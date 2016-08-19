/**
 * spaint: VoxelMarkerFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_VOXELMARKERFACTORY
#define H_SPAINT_VOXELMARKERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/VoxelMarker.h"

namespace spaint {

/**
 * \brief This struct can be used to construct voxel markers.
 */
struct VoxelMarkerFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a voxel marker.
   *
   * \param deviceType  The device on which the voxel marker should operate.
   * \return            The voxel marker.
   */
  static VoxelMarker_CPtr make_voxel_marker(ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
