/**
 * itmx: AckMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_ACKMESSAGE
#define H_ITMX_ACKMESSAGE

#include "MappingMessage.h"

#include <boost/shared_ptr.hpp>

namespace itmx {

/**
 * \brief An instance of this class represents a message containing the acknowledgement for a previously received message.
 *        The payload of this message is an integer that can be used to signal a status to the other party.
 */
class AckMessage : public MappingMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the size in bytes of the acknowledgement. */
  Segment m_acknowledgementSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs the header for a compressed RGB-D frame message.
   */
  AckMessage();

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes an AckMessage.
   */
  static boost::shared_ptr<AckMessage> make();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the status code.
   *
   * \return The status code.
   */
  int32_t extract_status_code() const;

  /**
   * \brief Sets the status code.
   *
   * \param statusCode The status code.
   */
  void set_status_code(int32_t statusCode);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<AckMessage> AckMessage_Ptr;
typedef boost::shared_ptr<const AckMessage> AckMessage_CPtr;

}

#endif
