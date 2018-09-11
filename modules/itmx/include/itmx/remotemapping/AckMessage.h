/**
 * itmx: AckMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_ACKMESSAGE
#define H_ITMX_ACKMESSAGE

#include <boost/cstdint.hpp>

#include "SimpleMessage.h"

namespace itmx {

/**
 * \brief An instance of this type represents a message containing the acknowledgement for a previously received message.
 */
struct AckMessage : SimpleMessage<int32_t>
{
  /**
   * \brief Constructs an acknowledgement message.
   */
  AckMessage()
  : SimpleMessage<int32_t>(0)
  {}
};

}

#endif
