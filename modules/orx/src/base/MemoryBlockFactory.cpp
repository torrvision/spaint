/**
 * orx: MemoryBlockFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "base/MemoryBlockFactory.h"

namespace orx {

//#################### SINGLETON IMPLEMENTATION ####################

MemoryBlockFactory::MemoryBlockFactory()
: m_deviceType(DEVICE_CUDA)
{}

MemoryBlockFactory& MemoryBlockFactory::instance()
{
  static MemoryBlockFactory s_instance;
  return s_instance;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void MemoryBlockFactory::set_device_type(DeviceType deviceType)
{
  m_deviceType = deviceType;
}

}
