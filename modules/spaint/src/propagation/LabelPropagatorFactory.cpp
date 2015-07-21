/**
 * spaint: LabelPropagatorFactory.cpp
 */

#include "propagation/LabelPropagatorFactory.h"

#include "propagation/cpu/LabelPropagator_CPU.h"

#ifdef WITH_CUDA
#include "propagation/cuda/LabelPropagator_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

LabelPropagator_CPtr LabelPropagatorFactory::make_label_propagator(size_t raycastResultSize, ITMLibSettings::DeviceType deviceType,
                                                                   float maxAngleBetweenNormals, float maxSquaredDistanceBetweenColours,
                                                                   float maxSquaredDistanceBetweenVoxels)
{
  LabelPropagator_CPtr propagator;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    propagator.reset(new LabelPropagator_CUDA(raycastResultSize, maxAngleBetweenNormals, maxSquaredDistanceBetweenColours, maxSquaredDistanceBetweenVoxels));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    propagator.reset(new LabelPropagator_CPU(raycastResultSize, maxAngleBetweenNormals, maxSquaredDistanceBetweenColours, maxSquaredDistanceBetweenVoxels));
  }

  return propagator;
}

}
