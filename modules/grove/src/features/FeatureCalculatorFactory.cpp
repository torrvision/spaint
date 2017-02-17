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

DA_RGBDPatchFeatureCalculator_CPtr FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(ITMLibSettings::DeviceType deviceType)
{
  DA_RGBDPatchFeatureCalculator_CPtr calculator;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    calculator.reset(new RGBDPatchFeatureCalculator_CUDA<Keypoint3DColour, RGBDPatchDescriptor>(true, 128, 0, 128, 128));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    calculator.reset(new RGBDPatchFeatureCalculator_CPU<Keypoint3DColour, RGBDPatchDescriptor>(true, 128, 0, 128, 128));
  }

  return calculator;
}

RGBPatchFeatureCalculator_CPtr FeatureCalculatorFactory::make_rgb_patch_feature_calculator(ITMLibSettings::DeviceType deviceType)
{
  RGBPatchFeatureCalculator_CPtr calculator;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    calculator.reset(new RGBDPatchFeatureCalculator_CUDA<Keypoint2D, RGBDPatchDescriptor>(false, 0, 0, 256, 0));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    calculator.reset(new RGBDPatchFeatureCalculator_CPU<Keypoint2D, RGBDPatchDescriptor>(false, 0, 0, 256, 0));
  }

  return calculator;
}

}
