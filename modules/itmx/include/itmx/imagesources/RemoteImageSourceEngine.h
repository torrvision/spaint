/**
 * itmx: RemoteImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_REMOTEIMAGESOURCEENGINE
#define H_ITMX_REMOTEIMAGESOURCEENGINE

#include "../base/ITMImagePtrTypes.h"
#include "../base/ITMObjectPtrTypes.h"
#include "../remotemapping/MappingServer.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to yield RGB-D images that have been obtained from a remote client.
 */
class RemoteImageSourceEngine : public InputSource::ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The ID of the remote client whose images are to be yielded. */
  int m_clientID;

  /** The mapping server used to communicate with remote clients. */
  MappingServer_Ptr m_mappingServer;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a remote image source engine.
   *
   * \param mappingServer The mapping server used to communicate with remote clients.
   * \param clientID      The ID of the remote client whose images are to be yielded.
   */
  RemoteImageSourceEngine(const MappingServer_Ptr& mappingServer, int clientID);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual ITMLib::ITMRGBDCalib getCalib() const;

  /** Override */
  virtual Vector2i getDepthImageSize() const;

  /** Override */
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize() const;

  /** Override */
  virtual bool hasImagesNow() const;

  /** Override */
  virtual bool hasMoreImages() const;
};

}

#endif
