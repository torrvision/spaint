/**
 * itmx: MappingClientData.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGCLIENTDATA
#define H_ITMX_MAPPINGCLIENTDATA

#include <ITMLib/Objects/Camera/ITMRGBDCalib.h>

#include <tvgutil/containers/PooledQueue.h>
#include <tvgutil/net/BasicClientData.h>

#include "RGBDFrameCompressor.h"

namespace itmx {

/**
 * \brief An instance of this struct can be used to hold the data associated with an individual client of a mapping server.
 */
struct MappingClientData : tvgutil::BasicClientData
{
  //#################### TYPEDEFS ####################

  typedef tvgutil::PooledQueue<RGBDFrameMessage_Ptr> RGBDFrameMessageQueue;
  typedef boost::shared_ptr<RGBDFrameMessageQueue> RGBDFrameMessageQueue_Ptr;

  //#################### PUBLIC VARIABLES ####################

  /** The calibration parameters of the camera associated with the client. */
  ITMLib::ITMRGBDCalib m_calib;

  /** A dummy frame message to consume messages that cannot be pushed onto the queue. */
  RGBDFrameMessage_Ptr m_dummyFrameMsg;

  /** The frame compressor for the client. */
  RGBDFrameCompressor_Ptr m_frameCompressor;

  /** A queue containing the RGB-D frame messages received from the client. */
  RGBDFrameMessageQueue_Ptr m_frameMessageQueue;

  /** A place in which to store compressed RGB-D frame messages. */
  boost::shared_ptr<CompressedRGBDFrameMessage> m_frameMessage;

  /** A place in which to store compressed RGB-D frame header messages. */
  CompressedRGBDFrameHeaderMessage m_headerMessage;

  /** A flag indicating whether or not the images associated with the first message in the queue have already been read. */
  bool m_imagesDirty;

  /** A flag indicating whether or not the pose associated with the first message in the queue has already been read. */
  bool m_poseDirty;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief TODO
   */
  MappingClientData();

  //#################### PUBLIC MEMBER FUNCTIONS ####################

  /**
   * \brief TODO
   */
  const Vector2i& get_depth_image_size() const;

  /**
   * \brief TODO
   */
  const Vector2i& get_rgb_image_size() const;
};

}

#endif
