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

class Settings : public ITMLib::ITMLibSettings, public tvgutil::SettingsContainer
{

};

typedef boost::shared_ptr<Settings> Settings_Ptr;
typedef boost::shared_ptr<const Settings> Settings_CPtr;

}

#endif
