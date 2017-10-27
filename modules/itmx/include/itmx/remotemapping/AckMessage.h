/**
 * itmx: AckMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_ACKMESSAGE
#define H_ITMX_ACKMESSAGE

#include <boost/cstdint.hpp>

#include "MappingMessage.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing the acknowledgement for a previously received message.
 *        The payload of this message is an integer that can be used to signal a status to the other party.
 */
class AckMessage : public MappingMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the status code. */
  Segment m_statusCodeSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an acknowledgement message.
   */
  AckMessage();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the status code from the message.
   *
   * \return  The status code.
   */
  int32_t extract_status_code() const;

  /**
   * \brief Sets the status code.
   *
   * \param statusCode  The status code.
   */
  void set_status_code(int32_t statusCode);
};

}

#endif
