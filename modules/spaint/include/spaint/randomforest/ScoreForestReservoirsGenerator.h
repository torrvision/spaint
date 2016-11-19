/**
 * spaint: ScoreForestReservoirsGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SCOREFORESTRESERVOIRSGENERATOR
#define H_SPAINT_SCOREFORESTRESERVOIRSGENERATOR

#include <ITMLib/Utils/ITMLibSettings.h>
#include "randomforest/interface/ExampleReservoirs.h"

namespace spaint
{
class ScoreForestReservoirsGenerator
{
public:
  static PositionReservoir_Ptr make_position_reservoir(
      ITMLib::ITMLibSettings::DeviceType deviceType, size_t reservoirCapacity,
      size_t nbReservoirs, uint32_t rngSeed = 42);
};
}

#endif
