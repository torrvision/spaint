/**
 * itmx: MappingMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/MappingMessage.h"

namespace itmx {

//#################### DESTRUCTOR ####################

MappingMessage::~MappingMessage() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

char *MappingMessage::get_data_ptr()
{
  return &m_data[0];
}

const char *MappingMessage::get_data_ptr() const
{
  return &m_data[0];
}

size_t MappingMessage::get_size() const
{
  return m_data.size();
}

}
