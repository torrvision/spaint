/**
 * spaint: SelectionTransformer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectiontransformers/interface/SelectionTransformer.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SelectionTransformer::SelectionTransformer(ITMLibSettings::DeviceType deviceType)
: m_deviceType(deviceType)
{}

//#################### DESTRUCTOR ####################

SelectionTransformer::~SelectionTransformer() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

SelectionTransformer::Selection *SelectionTransformer::transform_selection(const Selection& inputSelectionMB) const
{
  MemoryDeviceType memoryDeviceType = m_deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  Selection *outputSelectionMB = new Selection(compute_output_selection_size(inputSelectionMB), memoryDeviceType);
  transform_selection(inputSelectionMB, *outputSelectionMB);
  return outputSelectionMB;
}

}
