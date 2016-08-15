/**
 * spaintgui: SLAMSection.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINTGUI_SLAMSECTION
#define H_SPAINTGUI_SLAMSECTION

#include <boost/shared_ptr.hpp>

#include <InputSource/CompositeImageSourceEngine.h>
#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include <ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <ITMLib/Objects/Misc/ITMIMUCalibrator.h>
#include <ITMLib/Objects/RenderStates/ITMRenderState.h>

#include "SLAMState.h"
#include "TrackerType.h"

/**
 * \brief TODO
 */
class SLAMSection
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseMapper<spaint::SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMIMUCalibrator> IMUCalibrator_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTracker> ITMTracker_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef ITMLib::ITMScene<spaint::SpaintVoxel,ITMVoxelIndex> Scene;
  typedef boost::shared_ptr<Scene> Scene_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMLibSettings> Settings_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMTrackingController> TrackingController_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMViewBuilder> ViewBuilder_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The dense mapper. */
  DenseMapper_Ptr m_denseMapper;

  /** A pointer to a tracker that can detect tracking failures (if available). */
  spaint::FallibleTracker *m_fallibleTracker;

  /** The number of frames for which fusion has been run. */
  size_t m_fusedFramesCount;

  /** The engine used to provide input images to the fusion process. */
  CompositeImageSourceEngine_Ptr m_imageSourceEngine;

  /** The IMU calibrator. */
  IMUCalibrator_Ptr m_imuCalibrator;

  /**
   * A number of initial frames to fuse, regardless of their tracking quality.
   * Tracking quality can be poor in the first few frames, when there is only
   * a limited model against which to track. By forcibly fusing these frames,
   * we prevent poor tracking quality from stopping the reconstruction. After
   * these frames have been fused, only frames with a good tracking result will
   * be fused.
   */
  size_t m_initialFramesToFuse;

  /** The image into which depth input is read each frame. */
  ITMShortImage_Ptr m_inputRawDepthImage;

  /** The image into which RGB input is read each frame. */
  ITMUChar4Image_Ptr m_inputRGBImage;

  /** The remaining number of frames for which we need to achieve good tracking before we can add another keyframe. */
  size_t m_keyframeDelay;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The reconstructed scene. */
  Scene_Ptr m_scene;

  /** The tracker. */
  ITMTracker_Ptr m_tracker;

  /**
   * The parameters for the tracker (if any). For example, this would be the host on which the
   * Vicon software is running (e.g. "<IP address>:<port>") if we're using the Vicon tracker.
   */
  std::string m_trackerParams;

  /** The type of tracker to use. */
  TrackerType m_trackerType;

  /** The tracking controller. */
  TrackingController_Ptr m_trackingController;

  /** The current tracking state (containing the camera pose and additional tracking information used by InfiniTAM). */
  TrackingState_Ptr m_trackingState;

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   *
   * \param imageSourceEngine TODO
   * \param settings          The settings to use for InfiniTAM.
   * \param trackerType       TODO
   * \param trackerParams     TODO
   */
  SLAMSection(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, TrackerType trackerType, const std::string& trackerParams);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  ITMShortImage_CPtr get_input_raw_depth_image() const;

  /**
   * \brief TODO
   */
  ITMUChar4Image_CPtr get_input_rgb_image() const;

  /**
   * \brief TODO
   */
  const Scene_Ptr& get_scene();

  /**
   * \brief TODO
   */
  Vector2i get_tracked_image_size(const Vector2i& rgbImageSize, const Vector2i& depthImageSize) const;

  /**
   * \brief TODO
   */
  const TrackingState_Ptr& get_tracking_state();

  /** TODO */
  virtual bool run(SLAMState& state);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a hybrid tracker that refines the results of a primary tracker using ICP.
   *
   * \param primaryTracker    The primary tracker (e.g. a Rift or Vicon tracker).
   * \param settings          The settings to use for InfiniTAM.
   * \param scene             The scene.
   * \param rgbImageSize      The RGB image size.
   * \param depthImageSize    The depth image size.
   * \return                  The hybrid tracker.
   */
  ITMLib::ITMTracker *make_hybrid_tracker(ITMLib::ITMTracker *primaryTracker, const Settings_Ptr& settings, const Scene_Ptr& scene,
                                          const Vector2i& rgbImageSize, const Vector2i& depthImageSize) const;

  /**
   * \brief Sets up the tracker.
   *
   * \param settings          The settings to use for InfiniTAM.
   * \param scene             The scene.
   * \param rgbImageSize      The RGB image size.
   * \param depthImageSize    The depth image size.
   */
  void setup_tracker(const Settings_Ptr& settings, const Model::Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize);
};

#endif
