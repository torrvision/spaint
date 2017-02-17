/**
 * grove: FeatureCalculatorFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_GROVE_FEATURECALCULATORFACTORY
#define H_GROVE_FEATURECALCULATORFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/RGBDPatchFeatureCalculator.h"

namespace grove {

/**
 * \brief This struct can be used to construct feature calculators.
 */
struct FeatureCalculatorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a DA-RGBD patch feature calculator.
   *
   * \param deviceType The device on which the feature calculator should operate.
   */
  static DA_RGBDPatchFeatureCalculator_CPtr make_da_rgbd_patch_feature_calculator(ITMLib::ITMLibSettings::DeviceType deviceType);

  /**
   * \brief Makes a RGB patch feature calculator.
   *
   * \param deviceType The device on which the feature calculator should operate.
   */
  static RGBPatchFeatureCalculator_CPtr make_rgb_patch_feature_calculator(ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
