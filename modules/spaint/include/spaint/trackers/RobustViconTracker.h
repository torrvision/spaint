/**
 * spaint: RobustViconTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_ROBUSTVICONTRACKER
#define H_SPAINT_ROBUSTVICONTRACKER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Engine/ITMLowLevelEngine.h>
#include <ITMLib/Objects/ITMScene.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <rigging/SimpleCamera.h>

#include "ViconTracker.h"
#include "../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief TODO
 */
class RobustViconTracker : public ITMLib::Engine::ITMTracker
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::Engine::ITMLowLevelEngine> LowLevelEngine_CPtr;
  typedef boost::shared_ptr<ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> > Scene_Ptr;
  typedef boost::shared_ptr<const ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  boost::shared_ptr<ITMLib::Engine::ITMTracker> m_icpTracker;

  /** A flag recording whether or not we have temporarily lost tracking. */
  bool m_lostTracking;

  /** TODO */
  boost::shared_ptr<ViconTracker> m_viconTracker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  RobustViconTracker(const std::string& host, const std::string& subjectName, const Vector2i& trackedImageSize, const Settings_CPtr& settings,
                     const LowLevelEngine_CPtr& lowLevelEngine, const Scene_Ptr& scene);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not we have temporarily lost tracking.
   *
   * \return  true, if we have temporarily lost tracking, or false otherwise.
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
  static bool poses_are_similar(const rigging::SimpleCamera& cam1, const rigging::SimpleCamera& cam2,
                                double distanceThreshold = 0.3, double angleThreshold = 0.04);
};

}

#endif
