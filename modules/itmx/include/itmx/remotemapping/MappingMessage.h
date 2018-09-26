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
   * \brief TODO
   */
  ORUtils::SE3Pose read_pose(const Segment& segment) const;

  /**
   * \brief TODO
   */
  void write_pose(const ORUtils::SE3Pose& pose, const Segment& segment);

  //#################### PROTECTED STATIC MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief TODO
   */
  static size_t bytes_for_pose();
};

}

#endif
