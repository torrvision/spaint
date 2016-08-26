/**
 * spaint: RobustViconTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_ROBUSTVICONTRACKER
#define H_SPAINT_ROBUSTVICONTRACKER

#include <ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include <ITMLib/Utils/ITMLibSettings.h>

#include <rigging/SimpleCamera.h>

#include "ViconTracker.h"
#include "../util/SpaintVoxelScene.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to track the camera pose more robustly using a Vicon system.
 *
 * We first use the Vicon to obtain a coarse camera pose, and then refine it using ICP. We then test this
 * refined pose against both the pose from the previous frame and the Vicon pose, and fuse if and only if
 * the refined pose is relatively similar to both. This allows us to avoid fusing (a) when the camera is
 * moving too fast, and (b) when the ICP pose is likely to be unreliable.
 */
class RobustViconTracker : public FallibleTracker
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMLib::ITMLowLevelEngine> LowLevelEngine_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The ICP tracker. */
  boost::shared_ptr<ITMLib::ITMTracker> m_icpTracker;

  /** The Vicon tracker. */
  boost::shared_ptr<ViconTracker> m_viconTracker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a robust Vicon tracker.
   *
   * \param host              The host on which the Vicon software is running (e.g. "<IP address>:<port>").
   * \param subjectName       The name given to the camera subject in the Vicon software.
   * \param rgbImageSize      The RGB image size.
   * \param depthImageSize    The depth image size.
   * \param settings          The settings to use for InfiniTAM.
   * \param lowLevelEngine    The engine used to perform low-level image processing operations.
   * \param scene             The scene.
   */
  RobustViconTracker(const std::string& host, const std::string& subjectName, const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                     const Settings_CPtr& settings, const LowLevelEngine_CPtr& lowLevelEngine, const SpaintVoxelScene_Ptr& scene);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual bool requiresColourRendering() const;

  /** Override */
  virtual bool requiresDepthReliability() const;

  /** Override */
  virtual void TrackCamera(ITMLib::ITMTrackingState *trackingState, const ITMLib::ITMView *view);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes the angle between the two specified vectors.
   *
   * \param v1  The first vector.
   * \param v2  The second vector.
   * \return    The angle between the two vectors.
   */
  static double angle_between(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

  /**
   * \brief Determines whether or not the poses specified by two cameras are similar.
   *
   * Similarity is based on the distance between the positions of the cameras and the
   * angles between their corresponding axes.
   *
   * \param distanceThreshold The maximum distance (in metres) allowed between the camera positions for the poses to be considered similar.
   * \param angleThreshold    The maximum angle (in radians) allowed between corresponding camera axes for the poses to be considered similar.
   */
  static bool poses_are_similar(const rigging::SimpleCamera& cam1, const rigging::SimpleCamera& cam2,
                                double distanceThreshold = 0.3, double angleThreshold = 0.04);
};

}

#endif
