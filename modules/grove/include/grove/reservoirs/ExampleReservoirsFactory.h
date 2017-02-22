/**
 * grove: ExampleReservoirsFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSFACTORY
#define H_GROVE_EXAMPLERESERVOIRSFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ExampleReservoirs.h"

namespace grove
{

template <typename ExampleType, typename IndexType>
class ExampleReservoirsFactory
{
public:
  typedef ExampleReservoirs<ExampleType, IndexType> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;
  typedef boost::shared_ptr<const Reservoirs> Reservoirs_CPtr;

  static Reservoirs_Ptr make_reservoirs(
      ITMLib::ITMLibSettings::DeviceType deviceType, uint32_t reservoirCapacity,
      uint32_t reservoirCount, uint32_t rngSeed = 42);
};

}

#endif
