/**
 * itmx: DepthVisualiserFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "visualisation/DepthVisualiserFactory.h"
using namespace ITMLib;

#include "visualisation/cpu/DepthVisualiser_CPU.h"

#ifdef WITH_CUDA
#include "visualisation/cuda/DepthVisualiser_CUDA.h"
#endif

namespace itmx {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

DepthVisualiser_CPtr DepthVisualiserFactory::make_depth_visualiser(ITMLibSettings::DeviceType deviceType)
{
  DepthVisualiser_CPtr visualiser;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    visualiser.reset(new DepthVisualiser_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    visualiser.reset(new DepthVisualiser_CPU);
  }

  return visualiser;
}

}
