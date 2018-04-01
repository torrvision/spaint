/**
 * grove: ExampleReservoirsFactory.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "reservoirs/ExampleReservoirsFactory.h"

#include "reservoirs/cpu/ExampleReservoirs_CPU.h"

#ifdef WITH_CUDA
#include "reservoirs/cuda/ExampleReservoirs_CUDA.h"
#endif

namespace grove {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template <typename ExampleType>
typename ExampleReservoirsFactory<ExampleType>::Reservoirs_Ptr
ExampleReservoirsFactory<ExampleType>::make_reservoirs(uint32_t reservoirCount, uint32_t reservoirCapacity, DeviceType deviceType, uint32_t rngSeed)
{
  Reservoirs_Ptr reservoir;

  if(deviceType == DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    reservoir.reset(new ExampleReservoirs_CUDA<ExampleType>(reservoirCount, reservoirCapacity, rngSeed));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    reservoir.reset(new ExampleReservoirs_CPU<ExampleType>(reservoirCount, reservoirCapacity, rngSeed));
  }

  return reservoir;
}

}
