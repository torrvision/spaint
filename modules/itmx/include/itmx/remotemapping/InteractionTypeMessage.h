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
  IT_REQUESTGLOBALRENDER,

  /** An interaction in which the client sends a single RGB-D frame to the server. */
  IT_SENDFRAMETOSERVER,

  /** An unknown interaction (used when default constructing an interaction type message). */
  IT_UNKNOWN,
};

//#################### TYPES ####################

/**
 * \brief An instance of this class represents a message containing the way in which a mapping client next wants to interact with a mapping server.
 */
class InteractionTypeMessage : public SimpleMessage<InteractionType>
{
  //#################### CONSTRUCTORS #########################
public:
  /**
   * \brief Constructs an interaction type message.
   *
   * \param interactionType The interaction type to store in the message.
   */
  explicit InteractionTypeMessage(InteractionType interactionType = IT_UNKNOWN)
  {
    set_value(interactionType);
  }
};

}

#endif
