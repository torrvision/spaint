/**
 * spaint: PickerFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "picking/PickerFactory.h"
using namespace ITMLib;

#include "picking/cpu/Picker_CPU.h"

#ifdef WITH_CUDA
#include "picking/cuda/Picker_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Picker_CPtr PickerFactory::make_picker(ITMLibSettings::DeviceType deviceType)
{
  Picker_CPtr picker;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    picker.reset(new Picker_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    picker.reset(new Picker_CPU);
  }

  return picker;
}

}
