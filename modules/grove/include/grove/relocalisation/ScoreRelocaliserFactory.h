/**
 * grove: ScoreRelocaliserFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERFACTORY
#define H_GROVE_SCORERELOCALISERFACTORY

#include <ORUtils/DeviceType.h>

#include <tvgutil/misc/SettingsContainer.h>

#include "interface/ScoreRelocaliser.h"

namespace grove {

/**
 * \brief This struct can be used to create SCoRe relocalisers.
 */
struct ScoreRelocaliserFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a SCoRe relocaliser by loading a pre-trained forest from a file.
   *
   * \param forestFilename    The name of the file from which to load the pre-trained forest.
   * \param settings          The settings used to configure the relocaliser.
   * \param deviceType        The device on which the relocaliser should operate.
   * \param settingsNamespace The namespace associated with the settings that are specific to the SCoRe relocaliser.
   * \return                  The relocaliser.
   *
   * \throws std::runtime_error If the relocaliser cannot be created.
   */
  static ScoreRelocaliser_Ptr make_score_relocaliser(const std::string& forestFilename, const tvgutil::SettingsContainer_CPtr& settings, DeviceType deviceType, const std::string& settingsNamespace = "ScoreRelocaliser.");
};

}

#endif
