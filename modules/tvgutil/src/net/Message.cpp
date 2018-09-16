/**
 * tvgutil: Message.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "net/Message.h"

namespace tvgutil {

//#################### DESTRUCTOR ####################

Message::~Message() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

char *Message::get_data_ptr()
{
  return &m_data[0];
}

const char *Message::get_data_ptr() const
{
  return &m_data[0];
}

size_t Message::get_size() const
{
  return m_data.size();
}

}
