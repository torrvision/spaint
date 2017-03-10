/**
 * grove: FeatureCalculatorFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "features/FeatureCalculatorFactory.h"

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.h"

#ifdef WITH_CUDA
#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.h"
#endif

using namespace ITMLib;

namespace grove {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

DA_RGBDPatchFeatureCalculator_Ptr FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(ITMLibSettings::DeviceType deviceType)
{
  bool depthAdaptive = true;
  RGBDPatchFeatureCalculatorDifferenceType differenceType = RGBDPatchFeatureCalculatorDifferenceType::CENTRAL_DIFFERENCE;
  const uint32_t depthMinRadius = 1;       // as per Julien's code (was 2 / 2)
  const uint32_t depthMaxRadius = 130 / 2; // as per Julien's code
  const uint32_t depthFeatureCount = 128;
  const uint32_t depthFeatureOffset = 0;
  const uint32_t rgbMinRadius = 2;         // as per Julien's code
  const uint32_t rgbMaxRadius = 130;       // as per Julien's code
  const uint32_t rgbFeatureCount = 128;
  const uint32_t rgbFeatureOffset = 128;

  return make_custom_patch_feature_calculator<Keypoint3DColour,RGBDPatchDescriptor>(
        deviceType,
        depthAdaptive, differenceType, depthFeatureCount, depthFeatureOffset, depthMinRadius, depthMaxRadius,
        differenceType, rgbFeatureCount, rgbFeatureOffset, rgbMinRadius, rgbMaxRadius);
}

RGBPatchFeatureCalculator_Ptr FeatureCalculatorFactory::make_rgb_patch_feature_calculator(ITMLibSettings::DeviceType deviceType)
{
  bool depthAdaptive = false;
  RGBDPatchFeatureCalculatorDifferenceType differenceType = RGBDPatchFeatureCalculatorDifferenceType::CENTRAL_DIFFERENCE;
  const uint32_t depthMinRadius = 0;       // Unused
  const uint32_t depthMaxRadius = 0;       // Unused
  const uint32_t depthFeatureCount = 0;
  const uint32_t depthFeatureOffset = 0;
  const uint32_t rgbMinRadius = 2;         // as per Julien's code
  const uint32_t rgbMaxRadius = 130;       // as per Julien's code
  const uint32_t rgbFeatureCount = 256;
  const uint32_t rgbFeatureOffset = 0;

  return make_custom_patch_feature_calculator<Keypoint2D,RGBDPatchDescriptor>(
        deviceType,
        depthAdaptive, differenceType, depthFeatureCount, depthFeatureOffset, depthMinRadius, depthMaxRadius,
        differenceType, rgbFeatureCount, rgbFeatureOffset, rgbMinRadius, rgbMaxRadius);
}

}
