/**
 * itmx: MappingClientHandler.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGCLIENTHANDLER
#define H_ITMX_MAPPINGCLIENTHANDLER

#include <ITMLib/Objects/Camera/ITMRGBDCalib.h>

#include <tvgutil/containers/PooledQueue.h>
#include <tvgutil/misc/ExclusiveHandle.h>
#include <tvgutil/net/ClientHandler.h>

#include "RenderingRequestMessage.h"
#include "RGBDFrameCompressor.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to manage the connection to a mapping client.
 */
class MappingClientHandler : public tvgutil::ClientHandler
{
  //#################### TYPEDEFS ####################
private:
  typedef tvgutil::PooledQueue<RGBDFrameMessage_Ptr> RGBDFrameMessageQueue;
  typedef boost::shared_ptr<RGBDFrameMessageQueue> RGBDFrameMessageQueue_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The calibration parameters of the camera associated with the client. */
  ITMLib::ITMRGBDCalib m_calib;

  /** A dummy frame message to consume messages that cannot be pushed onto the queue. */
  RGBDFrameMessage_Ptr m_dummyFrameMessage;

  /** The frame compressor for the client. */
  RGBDFrameCompressor_Ptr m_frameCompressor;

  /** A place in which to store compressed RGB-D frame messages. */
  boost::shared_ptr<CompressedRGBDFrameMessage> m_frameMessage;

  /** A queue containing the RGB-D frame messages received from the client. */
  RGBDFrameMessageQueue_Ptr m_frameMessageQueue;

  /** A place in which to store compressed RGB-D frame header messages. */
  CompressedRGBDFrameHeaderMessage m_headerMessage;

  /** A flag indicating whether or not the images associated with the first message in the queue have already been read. */
  bool m_imagesDirty;

  /** A flag indicating whether or not the pose associated with the first message in the queue has already been read. */
  bool m_poseDirty;

  /** An optional image into which to render the scene for the client. */
  ORUChar4Image_Ptr m_renderedImage;

  /** The synchronisation mutex for the rendered image. */
  boost::mutex m_renderedImageMutex;

  /** An optional request from the client for the server to render the scene. */
  boost::optional<RenderingRequestMessage> m_renderingRequestMessage;

  /** The synchronisation mutex for the rendering request. */
  boost::mutex m_renderingRequestMutex;

  /** A place in which to store uncompressed RGB-D frame messages that can be used to send server-rendered images back to the client. */
  RGBDFrameMessage_Ptr m_renderingResponseMessage;

  /** The scene ID that is associated with the client. */
  std::string m_sceneID;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a handler for a mapping client.
   *
   * \param clientID          The ID used by the server to refer to the client.
   * \param sock              The socket used to communicate with the client.
   * \param shouldTerminate   Whether or not the server should terminate.
   */
  MappingClientHandler(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock, const boost::shared_ptr<const boost::atomic<bool> >& shouldTerminate);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the calibration parameters of the camera associated with the client.
   *
   * \return  The calibration parameters of the camera associated with the client.
   */
  const ITMLib::ITMRGBDCalib& get_calib() const;

  /**
   * \brief Gets the size of the depth images produced by the client.
   *
   * \return  The size of the depth images produced by the client.
   */
  const Vector2i& get_depth_image_size() const;

  /**
   * \brief Gets the queue containing the RGB-D frame messages received from the client.
   *
   * \return The queue containing the RGB-D frame messages received from the client.
   */
  const RGBDFrameMessageQueue_Ptr& get_frame_message_queue();

  /**
   * \brief Gets the image (if any) that the server has rendered for the client.
   *
   * \return  A handle providing exclusive access to the image (if any) that the server has rendered for the client.
   */
  tvgutil::ExclusiveHandle_Ptr<ORUChar4Image_Ptr>::Type get_rendered_image();

  /**
   * \brief Gets the current request (if any) from the client for the server to render the scene.
   *
   * \return  A handle providing exclusive access to the request (if any) from the client for the server to render the scene.
   */
  tvgutil::ExclusiveHandle_Ptr<boost::optional<RenderingRequestMessage> >::Type get_rendering_request();

  /**
   * \brief Gets the scene ID that is associated with the client.
   *
   * \return  The scene ID that is associated with the client.
   */
  const std::string& get_scene_id() const;

  /**
   * \brief Gets whether or not the images associated with the first message in the queue have already been read.
   *
   * \return  true, if the images associated with the first message in the queue have already been read, or false otherwise.
   */
  bool images_dirty() const;

  /**
   * \brief Gets whether or not the pose associated with the first message in the queue has already been read.
   *
   * \return  true, if the pose associated with the first message in the queue has already been read, or false otherwise.
   */
  bool pose_dirty() const;

  /**
   * \brief Gets the size of the colour images produced by the client.
   *
   * \return  The size of the colour images produced by the client.
   */
  const Vector2i& get_rgb_image_size() const;

  /** Override */
  virtual void run_iter();

  /** Override */
  virtual void run_post();

  /** Override */
  virtual void run_pre();

  /**
   * \brief Sets whether or not the images associated with the first message in the queue have already been read.
   *
   * \param imagesDirty Whether or not the images associated with the first message in the queue have already been read.
   */
  void set_images_dirty(bool imagesDirty);

  /**
   * \brief Sets whether or not the pose associated with the first message in the queue has already been read.
   *
   * \param poseDirty Whether or not the pose associated with the first message in the queue has already been read.
   */
  void set_pose_dirty(bool poseDirty);

  /**
   * \brief Sets the scene ID that is associated with the client.
   *
   * \param sceneID The scene ID that is associated with the client.
   */
  void set_scene_id(const std::string& sceneID);
};

}

#endif
