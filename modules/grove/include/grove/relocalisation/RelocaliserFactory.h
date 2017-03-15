/**
 * grove: RelocaliserFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RELOCALISERFACTORY
#define H_GROVE_RELOCALISERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ScoreRelocaliser.h"

namespace grove {

class RelocaliserFactory
{
public:
  static ScoreRelocaliser_Ptr make_score_relocaliser(ITMLib::ITMLibSettings::DeviceType deviceType, const std::string &forestFilename);
};

}

#endif
