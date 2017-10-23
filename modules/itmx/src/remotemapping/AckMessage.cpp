/**
 * itmx: AckMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/AckMessage.h"

#include <cstring>

namespace itmx {

//#################### CONSTRUCTORS ####################

AckMessage::AckMessage()
{
  m_statusCodeSegment = std::make_pair(0, sizeof(int32_t));
  m_data.resize(m_statusCodeSegment.second);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

AckMessage_Ptr AckMessage::make()
{
  return AckMessage_Ptr(new AckMessage());
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int32_t AckMessage::extract_status_code() const
{
  return *reinterpret_cast<const int32_t*>(&m_data[m_statusCodeSegment.first]);
}

void AckMessage::set_status_code(int32_t statusCode)
{
  memcpy(&m_data[m_statusCodeSegment.first], reinterpret_cast<const char*>(&statusCode), m_statusCodeSegment.second);
}

}
