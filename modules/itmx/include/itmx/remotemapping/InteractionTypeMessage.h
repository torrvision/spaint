/**
 * itmx: InteractionTypeMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_INTERACTIONTYPEMESSAGE
#define H_ITMX_INTERACTIONTYPEMESSAGE

#include "SimpleMessage.h"

namespace itmx {

//#################### ENUMERATIONS ####################

/**
 * \brief The values of this enumeration denote the different types of interaction that a mapping client can have with a mapping server.
 */
enum InteractionType
{
  /** An interaction in which the client asks the server to render and send across an image of the global map from a specified pose. */
  IT_REQUEST_GLOBAL_RENDER,

  /** The client wants to send a single RGB-D frame to the server. */
  IT_SEND_RGBD_FRAME,
};

//#################### TYPEDEFS ####################

/**
 * \brief An instance of this type represents a message containing the way in which a mapping client next wants to interact with a mapping server.
 */
typedef SimpleMessage<InteractionType> InteractionTypeMessage;

}

#endif
