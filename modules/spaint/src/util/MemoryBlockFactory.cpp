/**
 * spaint: MemoryBlockFactory.cpp
 */

#include "util/MemoryBlockFactory.h"

namespace spaint {

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
