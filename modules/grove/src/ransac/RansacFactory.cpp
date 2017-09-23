/**
 * grove: RansacFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/RansacFactory.h"
#include "ransac/cpu/PreemptiveRansac_CPU.h"

#ifdef WITH_CUDA
#include "ransac/cuda/PreemptiveRansac_CUDA.h"
#endif

using namespace ITMLib;

namespace grove {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

PreemptiveRansac_Ptr RansacFactory::make_preemptive_ransac(ITMLibSettings::DeviceType deviceType, const tvgutil::SettingsContainer_CPtr &settings)
{
  PreemptiveRansac_Ptr ransac;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    ransac.reset(new PreemptiveRansac_CUDA(settings));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    ransac.reset(new PreemptiveRansac_CPU(settings));
  }

  return ransac;
}

}
