/**
 * tvgutil: SimpleMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_TVGUTIL_SIMPLEMESSAGE
#define H_TVGUTIL_SIMPLEMESSAGE

#include "Message.h"

#include <cstring>

namespace tvgutil {

/**
 * \brief An instance of this class represents a message containing a single value of a specified type.
 */
template <typename T>
class SimpleMessage : public Message
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the message value. */
  Segment m_valueSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a simple message.
   *
   * \param value The initial message value (if any).
   */
  explicit SimpleMessage(const T& value = T())
  {
    m_valueSegment = std::make_pair(0, sizeof(T));
    m_data.resize(m_valueSegment.second);
    set_value(value);
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the message value.
   *
   * \return  The message value.
   */
  T extract_value() const
  {
    return *reinterpret_cast<const T*>(&m_data[m_valueSegment.first]);
  }

  /**
   * \brief Sets the message value.
   *
   * \param value The message value.
   */
  void set_value(T value)
  {
    memcpy(&m_data[m_valueSegment.first], reinterpret_cast<const char*>(&value), m_valueSegment.second);
  }
};

}

#endif
