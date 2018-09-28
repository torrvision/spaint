/**
 * itmx: MappingMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGMESSAGE
#define H_ITMX_MAPPINGMESSAGE

#include <ORUtils/SE3Pose.h>

#include <tvgutil/net/Message.h>

namespace itmx {

/**
 * \brief An instance of a class deriving from this one represents a message relating to remote mapping.
 */
class MappingMessage : public tvgutil::Message
{
  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Reads a 6D pose from the specified byte segment in the message.
   *
   * \param segment The byte segment from which to read the pose.
   * \return        The 6D pose.
   */
  ORUtils::SE3Pose read_pose(const Segment& segment) const;

  /**
   * \brief Writes a 6D pose into the specified byte segment in the message.
   *
   * \param pose    The 6D pose.
   * \param segment The byte segment into which to write it.
   */
  void write_pose(const ORUtils::SE3Pose& pose, const Segment& segment);

  //#################### PROTECTED STATIC MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Gets the number of bytes needed to store a 6D pose.
   *
   * \return  The number of bytes needed to store a 6D pose.
   */
  static size_t bytes_for_pose();
};

}

#endif
