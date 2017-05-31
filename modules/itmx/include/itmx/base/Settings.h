/**
 * itmx: Settings.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_SETTINGS
#define H_ITMX_SETTINGS

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMLibSettings.h>

#include <tvgutil/misc/SettingsContainer.h>

namespace itmx {

/**
 * \brief An instance of this class is used to configure the application, by adhering to the ITMLibSettings interface
 *        and additionally providing a map of non-typed parameters that can be accessed as in
 *        tvgutil::SettingsContainer.
 */
class Settings : public ITMLib::ITMLibSettings, public tvgutil::SettingsContainer
{
  // Nothing to add, just use whatever has been defined in the base classes.
};

}

#endif
