/**
 * grove: FeatureCalculatorFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_FEATURECALCULATORFACTORY
#define H_GROVE_FEATURECALCULATORFACTORY

#include <ORUtils/DeviceType.h>

#include "interface/RGBDPatchFeatureCalculator.h"

namespace grove {

/**
 * \brief This struct can be used to construct feature calculators.
 */
struct FeatureCalculatorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a patch feature calculator with custom parameters.
   *
   * \param deviceType           The device on which the feature calculator should operate.
   * \param depthAdaptive        Whether or not to compute the depth-normalised version of the features.
   * \param depthDifferenceType  The type of difference to use to compute depth features.
   * \param depthFeatureCount    The number of features to compute from the depth image.
   * \param depthFeatureOffset   The offset in the descriptor after which we store the depth features.
   * \param depthMinRadius       The minimum radius used to generate depth offsets.
   * \param depthMaxRadius       The maximum radius used to generate depth offsets.
   * \param rgbDifferenceType    The type of difference to use to compute RGB features.
   * \param rgbFeatureCount      The number of features to compute from the RGB image.
   * \param rgbFeatureOffset     The offset in the descriptor after which we store the RGB features.
   * \param rgbMinRadius         The minimum radius used to generate colour offsets.
   * \param rgbMaxRadius         The maximum radius used to generate colour offsets.
   * \return                     The feature calculator.
   *
   * \throws std::invalid_argument If depthFeatureCount + rgbFeatureCount > DescriptorType::FEATURE_COUNT, or if the offsets cause out-of-bounds access.
   */
  template <typename KeypointType, typename DescriptorType>
  static boost::shared_ptr<RGBDPatchFeatureCalculator<KeypointType, DescriptorType> > make_custom_patch_feature_calculator(
    DeviceType deviceType, bool depthAdaptive, RGBDPatchFeatureDifferenceType depthDifferenceType,
    uint32_t depthFeatureCount, uint32_t depthFeatureOffset, uint32_t depthMinRadius, uint32_t depthMaxRadius,
    RGBDPatchFeatureDifferenceType rgbDifferenceType, uint32_t rgbFeatureCount, uint32_t rgbFeatureOffset,
    uint32_t rgbMinRadius, uint32_t rgbMaxRadius
  );

  /**
   * \brief Makes a DA-RGBD patch feature calculator.
   *
   * \param deviceType  The device on which the feature calculator should operate.
   * \return            The feature calculator.
   */
  static DA_RGBDPatchFeatureCalculator_Ptr make_da_rgbd_patch_feature_calculator(DeviceType deviceType);

  /**
   * \brief Makes an RGB patch feature calculator.
   *
   * \param deviceType  The device on which the feature calculator should operate.
   * \return            The feature calculator.
   */
  static RGBPatchFeatureCalculator_Ptr make_rgb_patch_feature_calculator(DeviceType deviceType);
};

}

#endif
