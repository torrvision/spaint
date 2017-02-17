/**
 * spaint: FeatureCalculatorFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "features/FeatureCalculatorFactory.h"
using namespace ITMLib;

#include "features/cpu/RGBDPatchFeatureCalculator_CPU.h"
#include "features/cpu/VOPFeatureCalculator_CPU.h"

#ifdef WITH_CUDA
#include "features/cuda/RGBDPatchFeatureCalculator_CUDA.h"
#include "features/cuda/VOPFeatureCalculator_CUDA.h"
#endif

namespace spaint {

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

FeatureCalculator_CPtr FeatureCalculatorFactory::make_vop_feature_calculator(size_t maxVoxelLocationCount, size_t patchSize, float patchSpacing, size_t binCount,
                                                                             ITMLibSettings::DeviceType deviceType)
{
  FeatureCalculator_CPtr calculator;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    calculator.reset(new VOPFeatureCalculator_CUDA(maxVoxelLocationCount, patchSize, patchSpacing, binCount));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    calculator.reset(new VOPFeatureCalculator_CPU(maxVoxelLocationCount, patchSize, patchSpacing, binCount));
  }

  return calculator;
}

}
