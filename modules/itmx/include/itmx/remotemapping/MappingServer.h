/**
 * itmx: MappingServer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGSERVER
#define H_ITMX_MAPPINGSERVER

#include <map>
#include <set>
#include <vector>

#include <boost/atomic.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include <ITMLib/Objects/Camera/ITMRGBDCalib.h>

#include <tvgutil/boost/WrappedAsio.h>
#include <tvgutil/containers/PooledQueue.h>
#include <tvgutil/net/Server.h>

#include "RGBDFrameCompressor.h"

namespace itmx {

/**
 * \brief An instance of this struct contains all of the information associated with an individual client of a mapping server.
 */
struct MappingServerClient : tvgutil::DefaultClient
{
  //#################### TYPEDEFS ####################

  typedef tvgutil::PooledQueue<RGBDFrameMessage_Ptr> RGBDFrameMessageQueue;
  typedef boost::shared_ptr<RGBDFrameMessageQueue> RGBDFrameMessageQueue_Ptr;

  //#################### PUBLIC VARIABLES ####################

  /** The calibration parameters of the camera associated with the client. */
  ITMLib::ITMRGBDCalib m_calib;

  /** Whether or not the connection is still ok (effectively tracks whether or not the most recent read/write succeeded). */
  bool m_connectionOk;

  /** A dummy frame message to consume messages that cannot be pushed onto the queue. */
  RGBDFrameMessage_Ptr m_dummyFrameMsg;

  /** The frame compressor for the client. */
  RGBDFrameCompressor_Ptr m_frameCompressor;

  /** A queue containing the RGB-D frame messages received from the client. */
  RGBDFrameMessageQueue_Ptr m_frameMessageQueue;

  /** A flag indicating whether or not the images associated with the first message in the queue have already been read. */
  bool m_imagesDirty;

  /** A flag indicating whether or not the pose associated with the first message in the queue has already been read. */
  bool m_poseDirty;

  //#################### CONSTRUCTORS ####################

  MappingServerClient()
  : m_connectionOk(true),
    m_frameMessageQueue(new RGBDFrameMessageQueue(tvgutil::pooled_queue::PES_DISCARD)),
    m_imagesDirty(false),
    m_poseDirty(false)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################

  const Vector2i& get_depth_image_size() const
  {
    return m_calib.intrinsics_d.imgSize;
  }

  const Vector2i& get_rgb_image_size() const
  {
    return m_calib.intrinsics_rgb.imgSize;
  }
};

/**
 * \brief An instance of this class represents a server that can be used to communicate with remote mapping clients.
 */
class MappingServer : public tvgutil::Server<MappingServerClient>
{
  //#################### TYPEDEFS ####################
private:
  typedef tvgutil::PooledQueue<RGBDFrameMessage_Ptr> RGBDFrameMessageQueue;
  typedef boost::shared_ptr<RGBDFrameMessageQueue> RGBDFrameMessageQueue_Ptr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a mapping server.
   *
   * \param mode  The mode in which the server should run.
   * \param port  The port on which the server should listen for connections.
   */
  explicit MappingServer(Mode mode = SM_MULTI_CLIENT, int port = 7851);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to get the calibration parameters of the camera associated with the specified client.
   *
   * \param clientID  The ID of the client whose camera's calibration parameters we want to get.
   * \return          The calibration parameters of the camera associated with the specified client,
   *                  if the client is currently active, or default calibration parameters otherwise.
   */
  ITMLib::ITMRGBDCalib get_calib(int clientID) const;

  /**
   * \brief Attempts to get the size of depth image produced by the camera associated with the specified client.
   *
   * \param clientID  The ID of the client whose camera's depth image size we want to get.
   * \return          The size of depth image produced by the camera associated with the specified client,
   *                  if the client is currently active, or (0,0) otherwise.
   */
  Vector2i get_depth_image_size(int clientID) const;

  /**
   * \brief Attempts to get the next RGB-D image pair from the specified client.
   *
   * If the client has terminated, this will be a no-op.
   *
   * \param clientID  The ID of the client whose RGB-D images we want to get.
   * \param rgb       An image into which to copy the next RGB image from the client.
   * \param rawDepth  An image into which to copy the next depth image from the client.
   */
  void get_images(int clientID, ORUChar4Image *rgb, ORShortImage *rawDepth);

  /**
   * \brief Attempts to get the next pose from the specified client.
   *
   * If the client has terminated, this will be a no-op.
   *
   * \param clientID  The ID of the client whose pose we want to get.
   * \param pose      A pose into which to copy the next pose from the client.
   */
  void get_pose(int clientID, ORUtils::SE3Pose& pose);

  /**
   * \brief Attempts to get the size of RGB image produced by the camera associated with the specified client.
   *
   * \param clientID  The ID of the client whose camera's RGB image size we want to get.
   * \return          The size of RGB image produced by the camera associated with the specified client,
   *                  if the client is currently active, or (0,0) otherwise.
   */
  Vector2i get_rgb_image_size(int clientID) const;

  /**
   * \brief Gets whether or not the specified client is currently active and ready to yield an RGB-D frame.
   *
   * \param clientID  The ID of the client to check.
   * \return          true, if the client is currently active and ready to yield an RGB-D frame, or false otherwise.
   */
  bool has_images_now(int clientID) const;

  /**
   * \brief Gets whether or not the specified client is currently active and may still have more RGB-D frames to yield.
   *
   * \param clientID  The ID of the client to check.
   * \return          true, if the client is currently active and may still have more RGB-D frames to yield, or false otherwise.
   */
  bool has_more_images(int clientID) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void run_client_hook(int clientID, const Client_Ptr& client, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock);

  /** Override */
  virtual void setup_client_hook(int clientID, const Client_Ptr& client, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MappingServer> MappingServer_Ptr;
typedef boost::shared_ptr<const MappingServer> MappingServer_CPtr;

}

#endif
