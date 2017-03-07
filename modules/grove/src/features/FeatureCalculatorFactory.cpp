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
  DA_RGBDPatchFeatureCalculator_Ptr calculator;

  bool depthAdaptive = true;
  uint32_t depthFeatureCount = 128, depthFeatureOffset = 0, rgbFeatureCount = 128, rgbFeatureOffset = 128;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    calculator.reset(new RGBDPatchFeatureCalculator_CUDA<Keypoint3DColour,RGBDPatchDescriptor>(
      depthAdaptive, depthFeatureCount, depthFeatureOffset, rgbFeatureCount, rgbFeatureOffset
    ));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    calculator.reset(new RGBDPatchFeatureCalculator_CPU<Keypoint3DColour,RGBDPatchDescriptor>(
      depthAdaptive, depthFeatureCount, depthFeatureOffset, rgbFeatureCount, rgbFeatureOffset
    ));
  }

  return calculator;
}

RGBPatchFeatureCalculator_Ptr FeatureCalculatorFactory::make_rgb_patch_feature_calculator(ITMLibSettings::DeviceType deviceType)
{
  RGBPatchFeatureCalculator_Ptr calculator;

  bool depthAdaptive = false;
  uint32_t depthFeatureCount = 0, depthFeatureOffset = 0, rgbFeatureCount = 256, rgbFeatureOffset = 0;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    calculator.reset(new RGBDPatchFeatureCalculator_CUDA<Keypoint2D,RGBDPatchDescriptor>(
      depthAdaptive, depthFeatureCount, depthFeatureOffset, rgbFeatureCount, rgbFeatureOffset
    ));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    calculator.reset(new RGBDPatchFeatureCalculator_CPU<Keypoint2D,RGBDPatchDescriptor>(
      depthAdaptive, depthFeatureCount, depthFeatureOffset, rgbFeatureCount, rgbFeatureOffset
    ));
  }

  return calculator;
}

}
