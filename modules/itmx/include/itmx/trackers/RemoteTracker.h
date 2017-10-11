/**
 * itmx: RemoteTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_REMOTETRACKER
#define H_ITMX_REMOTETRACKER

#include <ITMLib/Trackers/Interface/ITMTracker.h>

#include "../remotemapping/MappingServer.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to yield poses that have been obtained from a remote client.
 */
class RemoteTracker : public ITMLib::ITMTracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The ID of the remote client whose poses are to be yielded. */
  int m_clientID;

  /** The mapping server used to communicate with remote clients. */
  MappingServer_Ptr m_mappingServer;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a remote tracker.
   *
   * \param mappingServer The mapping server used to communicate with remote clients.
   * \param clientID      The ID of the remote client whose poses are to be yielded.
   */
  RemoteTracker(const MappingServer_Ptr& mappingServer, int clientID);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual bool requiresColourRendering() const;

  /** Override */
  virtual bool requiresDepthReliability() const;

  /** Override */
  virtual bool requiresPointCloudRendering() const;

  /** Override */
  virtual void TrackCamera(ITMLib::ITMTrackingState *trackingState, const ITMLib::ITMView *view);
};

}

#endif
