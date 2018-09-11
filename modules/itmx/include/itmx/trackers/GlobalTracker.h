/**
 * itmx: GlobalTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_GLOBALTRACKER
#define H_ITMX_GLOBALTRACKER

#include "../base/ITMObjectPtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to decorate an existing tracker to yield camera poses in a global coordinate frame.
 */
class GlobalTracker : public ITMLib::ITMTracker
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The global pose of the first frame in the sequence. */
  ORUtils::SE3Pose m_initialPose;

  /** The tracker to decorate. */
  Tracker_Ptr m_tracker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a global tracker.
   *
   * \param tracker     The tracker to decorate.
   * \param initialPose The global pose of the first frame in the sequence.
   */
  GlobalTracker(const Tracker_Ptr& tracker, const ORUtils::SE3Pose& initialPose);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual bool CanKeepTracking() const;

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
