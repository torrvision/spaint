/**
 * spaint: ScoreForestReservoirsGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/ScoreForestReservoirsGenerator.h"
#include "randomforest/cpu/ExampleReservoirs_CPU.h"

#ifdef WITH_CUDA
#include "randomforest/cuda/ExampleReservoirs_CUDA.h"
#endif

namespace spaint
{
PositionReservoir_Ptr ScoreForestReservoirsGenerator::make_position_reservoir(
    ITMLib::ITMLibSettings::DeviceType deviceType, size_t reservoirCapacity,
    size_t nbReservoirs, uint32_t rngSeed)
{
  PositionReservoir_Ptr reservoir;

  if (deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    reservoir.reset(
        new ExampleReservoirs_CUDA<PositionColourExample, RGBDPatchFeature,
            LeafIndices>(reservoirCapacity, nbReservoirs, rngSeed));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    reservoir.reset(
        new ExampleReservoirs_CPU<PositionColourExample, RGBDPatchFeature,
            LeafIndices>(reservoirCapacity, nbReservoirs, rngSeed));
  }

  return reservoir;
}
}
