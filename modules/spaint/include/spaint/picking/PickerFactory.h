/**
 * spaint: PickerFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PICKERFACTORY
#define H_SPAINT_PICKERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/Picker.h"

namespace spaint {

/**
 * \brief This struct can be used to construct pickers.
 */
struct PickerFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a picker.
   *
   * \param deviceType  The device on which the picker should operate.
   * \return            The picker.
   */
  static Picker_CPtr make_picker(ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
