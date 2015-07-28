/**
 * spaint: RefineWithICPTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_REFINEWITHICPTRACKER
#define H_SPAINT_REFINEWITHICPTRACKER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Engine/ITMLowLevelEngine.h>
#include <ITMLib/Objects/ITMScene.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <rigging/SimpleCamera.h>

#include "util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief TODO
 */
class RefineWithICPTracker : public ITMLib::Engine::ITMTracker
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::Engine::ITMLowLevelEngine> LowLevelEngine_CPtr;
  typedef boost::shared_ptr<ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> > Scene_Ptr;
  typedef boost::shared_ptr<const ITMLibSettings> Settings_CPtr;
  typedef boost::shared_ptr<ITMLib::Engine::ITMTracker> Tracker_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  Tracker_Ptr m_baseTracker;

  /** TODO */
  bool m_icpSucceeded;

  /** TODO */
  Tracker_Ptr m_icpTracker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  RefineWithICPTracker(ITMLib::Engine::ITMTracker *baseTracker, const Vector2i& trackedImageSize, const Settings_CPtr& settings,
                       const LowLevelEngine_CPtr& lowLevelEngine, const Scene_Ptr& scene);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  bool lost_tracking() const;

  /** Override */
  virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

  /** Override */
  virtual void UpdateInitialPose(ITMTrackingState *trackingState);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  static double angle_between(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

  /**
   * \brief TODO
   */
  static bool poses_are_similar(const rigging::SimpleCamera& cam1, const rigging::SimpleCamera& cam2, double distanceThreshold);
};

}

#endif
