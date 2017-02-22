/**
 * grove: ExampleReservoirsFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "reservoirs/ExampleReservoirsFactory.h"

#include "reservoirs/cpu/ExampleReservoirs_CPU.h"

#ifdef WITH_CUDA
#include "reservoirs/cuda/ExampleReservoirs_CUDA.h"
#endif

namespace grove {

template <typename ExampleType, typename IndexType>
typename ExampleReservoirsFactory<ExampleType, IndexType>::Reservoirs_Ptr ExampleReservoirsFactory<ExampleType, IndexType>::make_reservoirs(
    ITMLib::ITMLibSettings::DeviceType deviceType, uint32_t reservoirCapacity,
    uint32_t reservoirCount, uint32_t rngSeed)
{
  Reservoirs_Ptr reservoir;

  if (deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    reservoir.reset(
        new ExampleReservoirs_CUDA<ExampleType, IndexType>(reservoirCapacity, reservoirCount, rngSeed));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    reservoir.reset(
        new ExampleReservoirs_CPU<ExampleType, IndexType>(reservoirCapacity, reservoirCount, rngSeed));
  }

  return reservoir;
}

}
