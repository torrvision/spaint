/**
 * grove: PreemptiveRansacFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/PreemptiveRansacFactory.h"
using namespace tvgutil;

#include "ransac/cpu/PreemptiveRansac_CPU.h"

#ifdef WITH_CUDA
#include "ransac/cuda/PreemptiveRansac_CUDA.h"
#endif

namespace grove {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

PreemptiveRansac_Ptr PreemptiveRansacFactory::make_preemptive_ransac(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace, DeviceType deviceType)
{
  PreemptiveRansac_Ptr ransac;

  if(deviceType == DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    ransac.reset(new PreemptiveRansac_CUDA(settings, settingsNamespace));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    ransac.reset(new PreemptiveRansac_CPU(settings, settingsNamespace));
  }

  return ransac;
}

}
