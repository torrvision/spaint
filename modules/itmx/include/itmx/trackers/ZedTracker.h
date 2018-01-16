/**
 * itmx: ZedTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_ZEDTRACKER
#define H_ITMX_ZEDTRACKER

#include <ITMLib/Trackers/Interface/ITMTracker.h>

#include "../util/ZedCamera.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to yield camera poses that have been obtained from a Zed camera.
 */
class ZedTracker : public ITMLib::ITMTracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The Zed camera from which to obtain the camera poses. */
  ZedCamera_Ptr m_camera;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Zed tracker.
   *
   * \param camera  The Zed camera from which to obtain the camera poses.
   */
  explicit ZedTracker(const ZedCamera_Ptr& camera);

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

  /** Override */
  virtual void UpdateInitialPose(ITMLib::ITMTrackingState *trackingState);
};

}

#endif
