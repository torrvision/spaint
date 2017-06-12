/**
 * itmx: Settings.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_SETTINGS
#define H_ITMX_SETTINGS

#include <ITMLib/Utils/ITMLibSettings.h>

#include <tvgutil/misc/SettingsContainer.h>

namespace itmx {

/**
 * \brief An instance of this struct can be used to store the settings for an itmx-based application.
 *
 * We use multiple inheritance to combine an instance of ITMLibSettings with a settings container that
 * allows us to store additional settings that were not present in core InfiniTAM.
 */
struct Settings : ITMLib::ITMLibSettings, tvgutil::SettingsContainer {};

}

#endif
