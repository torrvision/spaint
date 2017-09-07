/**
 * grove: ExampleReservoirsFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSFACTORY
#define H_GROVE_EXAMPLERESERVOIRSFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ExampleReservoirs.h"

namespace grove {

/**
 * \brief An instantiation of this struct template can be used to construct sets of example reservoirs.
 */
template <typename ExampleType>
struct ExampleReservoirsFactory
{
  //#################### TYPEDEFS ####################

  typedef ExampleReservoirs<ExampleType> Reservoirs;
  typedef boost::shared_ptr<Reservoirs> Reservoirs_Ptr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a set of example reservoirs.
   *
   * \param reservoirCount    The number of reservoirs to create.
   * \param reservoirCapacity The capacity (maximum size) of each reservoir.
   * \param deviceType        The device on which the example reservoirs should be stored.
   * \param rngSeed           The seed for the random number generator.
   */
  static Reservoirs_Ptr make_reservoirs(uint32_t reservoirCount, uint32_t reservoirCapacity, ITMLib::ITMLibSettings::DeviceType deviceType, uint32_t rngSeed = 42);
};

}

#endif
