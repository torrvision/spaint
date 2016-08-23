/**
 * spaint: SLAMComponent.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SLAMCOMPONENT
#define H_SPAINT_SLAMCOMPONENT

#include <InputSource/CompositeImageSourceEngine.h>
#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include <ITMLib/Engines/ViewBuilding/Interface/ITMViewBuilder.h>
#include <ITMLib/Objects/Misc/ITMIMUCalibrator.h>
#include <RelocLib/PoseDatabase.h>
#include <RelocLib/Relocaliser.h>

#include "SLAMContext.h"
#include "../trackers/FallibleTracker.h"
#include "../trackers/TrackerType.h"
#include "../util/ITMImagePtrTypes.h"
#include "../util/SpaintScene.h"

namespace spaint {

/**
 * \brief An instance of this pipeline component can be used to perform simultaneous localisation and mapping (SLAM).
 */
class SLAMComponent
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<InputSource::CompositeImageSourceEngine> CompositeImageSourceEngine_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMDenseMapper<SpaintVoxel,ITMVoxelIndex> > DenseMapper_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMIMUCalibrator> IMUCalibrator_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMTracker> ITMTracker_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMLowLevelEngine> LowLevelEngine_Ptr;
  typedef boost::shared_ptr<RelocLib::PoseDatabase> PoseDatabase_Ptr;
  typedef boost::shared_ptr<RelocLib::Relocaliser> Relocaliser_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMRenderState> RenderState_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;
  typedef boost::shared_ptr<const ITMLib::ITMLibSettings> Settings_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingController> TrackingController_Ptr;
  typedef boost::shared_ptr<const ITMLib::ITMTrackingController> TrackingController_CPtr;
  typedef boost::shared_ptr<ITMLib::ITMTrackingState> TrackingState_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMView> View_Ptr;
  typedef boost::shared_ptr<ITMLib::ITMViewBuilder> ViewBuilder_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The shared context needed for SLAM. */
  SLAMContext_Ptr m_context;

  /** The dense mapper. */
  DenseMapper_Ptr m_denseMapper;

  /** A pointer to a tracker that can detect tracking failures (if available). */
  FallibleTracker *m_fallibleTracker;

  /** The number of frames for which fusion has been run. */
  size_t m_fusedFramesCount;

  /** Whether or not the user wants fusion to be run. */
  bool m_fusionEnabled;

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

  /** The remaining number of frames for which we need to achieve good tracking before we can add another keyframe. */
  size_t m_keyframeDelay;

  /** The engine used to perform low-level image processing operations. */
  LowLevelEngine_Ptr m_lowLevelEngine;

  /** The database of previous poses for relocalisation. */
  PoseDatabase_Ptr m_poseDatabase;

  /** The relocaliser. */
  Relocaliser_Ptr m_relocaliser;

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

  /** The view builder. */
  ViewBuilder_Ptr m_viewBuilder;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a SLAM component.
   *
   * \param context           The shared context needed for SLAM.
   * \param imageSourceEngine The engine used to provide input images to the fusion process.
   * \param trackerType       The type of tracker to use.
   * \param trackerParams     The parameters for the tracker (if any).
   */
  SLAMComponent(const SLAMContext_Ptr& context, const CompositeImageSourceEngine_Ptr& imageSourceEngine, TrackerType trackerType, const std::string& trackerParams);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not the user wants fusion to be run.
   *
   * \return  true, if the user wants fusion to be run, or false otherwise.
   */
  bool get_fusion_enabled() const;

  /**
   * \brief Runs the SLAM component.
   */
  bool run();

  /**
   * \brief Sets whether or not the user wants fusion to be run.
   *
   * Note: Just because the user wants fusion to be run doesn't mean that it necessarily will be on every frame.
   *       In particular, we prevent fusion when we know we have lost tracking, regardless of this setting.
   *
   * \param fusionEnabled Whether or not the user wants fusion to be run.
   */
  void set_fusion_enabled(bool fusionEnabled);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a hybrid tracker that refines the results of a primary tracker using ICP.
   *
   * \param primaryTracker  The primary tracker (e.g. a Rift or Vicon tracker).
   * \param rgbImageSize    The RGB image size.
   * \param depthImageSize  The depth image size.
   * \return                The hybrid tracker.
   */
  ITMLib::ITMTracker *make_hybrid_tracker(ITMLib::ITMTracker *primaryTracker, const Vector2i& rgbImageSize, const Vector2i& depthImageSize) const;

  /**
   * \brief Sets up the tracker.
   *
   * \param rgbImageSize    The RGB image size.
   * \param depthImageSize  The depth image size.
   */
  void setup_tracker(const Vector2i& rgbImageSize, const Vector2i& depthImageSize);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SLAMComponent> SLAMComponent_Ptr;

}

#endif
