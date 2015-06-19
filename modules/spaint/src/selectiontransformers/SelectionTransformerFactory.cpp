/**
 * spaint: SelectionTransformerFactory.cpp
 */

#include "selectiontransformers/SelectionTransformerFactory.h"

#include "selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"

#ifdef WITH_CUDA
#include "selectiontransformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

SelectionTransformer_Ptr SelectionTransformerFactory::make_voxel_to_cube(int radius, ITMLibSettings::DeviceType deviceType)
{
  SelectionTransformer_Ptr transformer;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    transformer.reset(new VoxelToCubeSelectionTransformer_CUDA(radius));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    transformer.reset(new VoxelToCubeSelectionTransformer_CPU(radius));
  }

  return transformer;
}

}
