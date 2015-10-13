/**
 * spaint: LabelSmootherFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "smoothing/LabelSmootherFactory.h"

#include "smoothing/cpu/LabelSmoother_CPU.h"

#ifdef WITH_CUDA
#include "smoothing/cuda/LabelSmoother_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

LabelSmoother_CPtr LabelSmootherFactory::make_label_smoother(size_t maxLabelCount,  ITMLibSettings::DeviceType deviceType, float maxSquaredDistanceBetweenVoxels)
{
  LabelSmoother_CPtr smoother;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    smoother.reset(new LabelSmoother_CUDA(maxLabelCount, maxSquaredDistanceBetweenVoxels));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    smoother.reset(new LabelSmoother_CPU(maxLabelCount, maxSquaredDistanceBetweenVoxels));
  }

  return smoother;
}

}
