/**
 * spaint: VoxelMarkerFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "markers/VoxelMarkerFactory.h"
using namespace ITMLib;

#include "markers/cpu/VoxelMarker_CPU.h"

#ifdef WITH_CUDA
#include "markers/cuda/VoxelMarker_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

VoxelMarker_CPtr VoxelMarkerFactory::make_voxel_marker(DeviceType deviceType)
{
  VoxelMarker_CPtr marker;

  if(deviceType == DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    marker.reset(new VoxelMarker_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    marker.reset(new VoxelMarker_CPU);
  }

  return marker;
}

}
