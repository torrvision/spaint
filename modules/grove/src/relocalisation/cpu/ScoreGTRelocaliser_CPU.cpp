/**
 * grove: ScoreGTRelocaliser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/cpu/ScoreGTRelocaliser_CPU.h"
using namespace ORUtils;
using namespace tvgutil;

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreGTRelocaliser_CPU::ScoreGTRelocaliser_CPU(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace)
: ScoreGTRelocaliser(settings, settingsNamespace, DEVICE_CPU)
{}

}
