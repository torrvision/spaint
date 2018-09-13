/**
 * itmx: MappingServer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGSERVER
#define H_ITMX_MAPPINGSERVER

#include <tvgutil/net/Server.h>

#include "MappingClientHandler.h"

namespace itmx {

/**
 * \brief An instance of this class represents a server that can be used to communicate with remote mapping clients.
 */
class MappingServer : public tvgutil::Server<MappingClientHandler>
{
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
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MappingServer> MappingServer_Ptr;
typedef boost::shared_ptr<const MappingServer> MappingServer_CPtr;

}

#endif
