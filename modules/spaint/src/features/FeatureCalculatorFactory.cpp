/**
 * spaint: FeatureCalculatorFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "features/FeatureCalculatorFactory.h"

#include "features/cpu/VOPFeatureCalculator_CPU.h"

#ifdef WITH_CUDA
#include "features/cuda/VOPFeatureCalculator_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

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
