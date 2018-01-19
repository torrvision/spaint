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
 *        The payload of this message is an integer that can be used to signal a status to the other party.
 */
typedef SimpleMessage<int32_t> AckMessage;

}

#endif
