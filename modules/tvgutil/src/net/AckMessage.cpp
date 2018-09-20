/**
 * tvgutil: AckMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "net/AckMessage.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

AckMessage::AckMessage()
: SimpleMessage<int32_t>(0)
{}

}
