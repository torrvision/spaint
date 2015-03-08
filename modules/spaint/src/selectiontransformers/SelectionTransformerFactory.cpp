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

SelectionTransformer_CPtr SelectionTransformerFactory::make_voxel_to_cube(int radius, ITMLibSettings::DeviceType deviceType)
{
  boost::shared_ptr<const SelectionTransformer> transformer;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    transformer.reset(new VoxelToCubeSelectionTransformer_CUDA(radius));
#else
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
