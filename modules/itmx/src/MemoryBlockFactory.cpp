/**
 * itmx: MemoryBlockFactory.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "MemoryBlockFactory.h"
using namespace ITMLib;

namespace itmx {

//#################### SINGLETON IMPLEMENTATION ####################

MemoryBlockFactory::MemoryBlockFactory()
: m_deviceType(ITMLibSettings::DEVICE_CUDA)
{}

MemoryBlockFactory& MemoryBlockFactory::instance()
{
  static MemoryBlockFactory s_instance;
  return s_instance;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void MemoryBlockFactory::set_device_type(ITMLibSettings::DeviceType deviceType)
{
  m_deviceType = deviceType;
}

}
