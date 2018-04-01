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
 * \brief This class allows the construction of a ScoreRelocaliser.
 */
class ScoreRelocaliserFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Create a ScoreRelocaliser of the appropriate type.
   *
   * \param deviceType     The device type.
   * \param settings       A pointer to the settings used to configure the relocaliser.
   * \param forestFilename The path to a file containing the structure of a pretrained relocalisation forest.
   *
   * \return An instance of a ScoreRelocaliser.
   *
   * \throws std::runtime_error If the relocaliser cannot be created.
   */
  static ScoreRelocaliser_Ptr make_score_relocaliser(DeviceType deviceType, const tvgutil::SettingsContainer_CPtr& settings, const std::string& forestFilename);
};

}

#endif
