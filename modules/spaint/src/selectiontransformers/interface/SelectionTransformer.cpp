/**
 * spaint: SelectionTransformer.cpp
 */

#include "selectiontransformers/interface/SelectionTransformer.h"

namespace spaint {

//#################### DESTRUCTOR ####################

SelectionTransformer::~SelectionTransformer() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

SelectionTransformer::Selection *SelectionTransformer::transform_selection(const Selection& inputSelectionMB, ITMLibSettings::DeviceType deviceType) const
{
  MemoryDeviceType memoryDeviceType = deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  Selection *outputSelectionMB = new Selection(compute_output_selection_size(inputSelectionMB), memoryDeviceType);
  transform_selection(inputSelectionMB, *outputSelectionMB);
  return outputSelectionMB;
}

}
