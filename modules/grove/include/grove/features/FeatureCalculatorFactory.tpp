/**
 * grove: FeatureCalculatorFactory.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "FeatureCalculatorFactory.h"
#include "cpu/RGBDPatchFeatureCalculator_CPU.h"

#ifdef WITH_CUDA
#include "cuda/RGBDPatchFeatureCalculator_CUDA.h"
#endif

namespace grove {

template <typename KeypointType, typename DescriptorType>
boost::shared_ptr<RGBDPatchFeatureCalculator<KeypointType, DescriptorType> > FeatureCalculatorFactory::make_custom_patch_feature_calculator(ITMLib::ITMLibSettings::DeviceType deviceType,
    bool depthAdaptive,
    RGBDPatchFeatureCalculatorDifferenceType depthDifferenceType,
    uint32_t depthFeatureCount,
    uint32_t depthFeatureOffset,
    uint32_t depthMinRadius,
    uint32_t depthMaxRadius,
    RGBDPatchFeatureCalculatorDifferenceType rgbDifferenceType,
    uint32_t rgbFeatureCount,
    uint32_t rgbFeatureOffset,
    uint32_t rgbMinRadius,
    uint32_t rgbMaxRadius)
{
  boost::shared_ptr<RGBDPatchFeatureCalculator<KeypointType,DescriptorType> > calculator;

  if(deviceType == ITMLib::ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    calculator.reset(new RGBDPatchFeatureCalculator_CUDA<KeypointType,DescriptorType>(
      depthAdaptive, depthDifferenceType, depthFeatureCount, depthFeatureOffset, depthMinRadius, depthMaxRadius,
                     rgbDifferenceType, rgbFeatureCount, rgbFeatureOffset, rgbMinRadius, rgbMaxRadius
    ));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    calculator.reset(new RGBDPatchFeatureCalculator_CPU<KeypointType,DescriptorType>(
      depthAdaptive, depthDifferenceType, depthFeatureCount, depthFeatureOffset, depthMinRadius, depthMaxRadius,
                     rgbDifferenceType, rgbFeatureCount, rgbFeatureOffset, rgbMinRadius, rgbMaxRadius
    ));
  }

  return calculator;
}

}
