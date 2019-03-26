/**
 * grove: ScoreGTRelocaliser_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCOREGTRELOCALISER_CPU
#define H_GROVE_SCOREGTRELOCALISER_CPU

#include "../interface/ScoreGTRelocaliser.h"

namespace grove {

/**
 * \brief An instance of this class can be used to relocalise a camera in a 3D scene on the CPU, using known ground truth correspondences.
 */
class ScoreGTRelocaliser_CPU : public ScoreGTRelocaliser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a ground truth SCoRe relocaliser on the CPU.
   *
   * \param settings          The settings used to configure the relocaliser.
   * \param settingsNamespace The namespace associated with the settings that are specific to the relocaliser.
   */
  ScoreGTRelocaliser_CPU(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace);
};

}

#endif
