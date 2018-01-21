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
  /** An interaction in which the client asks the server to send its scene rendering for that client. */
  IT_GETRENDERING,

  /** An interaction in which the client sends a single RGB-D frame to the server. */
  IT_SENDFRAME,

  /** An interaction in which the client sends a new rendering pose to the server. */
  IT_UPDATERENDERINGPOSE,
};

//#################### TYPES ####################

/**
 * \brief An instance of this type represents a message containing the way in which a mapping client next wants to interact with a mapping server.
 */
typedef SimpleMessage<InteractionType> InteractionTypeMessage;

}

#endif
