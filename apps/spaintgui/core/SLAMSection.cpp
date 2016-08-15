/**
 * spaintgui: SLAMSection.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "SLAMSection.h"
using namespace spaint;

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>
using namespace ITMLib;
using namespace ORUtils;

//#################### CONSTRUCTORS ####################

SLAMSection::SLAMSection(const CompositeImageSourceEngine_Ptr& imageSourceEngine, const Settings_Ptr& settings, TrackerType trackerType, const std::string& trackerParams)
: m_fusedFramesCount(0),
  m_imageSourceEngine(imageSourceEngine),
  m_initialFramesToFuse(50), // FIXME: This value should be passed in rather than hard-coded.
  m_keyframeDelay(0),
  m_trackerParams(trackerParams),
  m_trackerType(trackerType)
{
  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the RGB and raw depth images into which input is to be read each frame.
  m_inputRGBImage.reset(new ITMUChar4Image(rgbImageSize, true, true));
  m_inputRawDepthImage.reset(new ITMShortImage(depthImageSize, true, true));

  // Set up the low-level engine.
  m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType));

  // Set up the view builder.
  m_viewBuilder.reset(ITMViewBuilderFactory::MakeViewBuilder(&m_imageSourceEngine->getCalib(), settings->deviceType));

  // Set up the scene.
  MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  m_scene.reset(new Scene(&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType));

  // Set up the dense mapper.
  m_denseMapper.reset(new ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>(settings.get()));
  m_denseMapper->ResetScene(m_scene.get());

  // Set up the tracker and the tracking controller.
  setup_tracker(settings, m_scene, rgbImageSize, depthImageSize);
  m_trackingController.reset(new ITMTrackingController(m_tracker.get(), settings.get()));
  m_trackingState.reset(new ITMTrackingState(get_tracked_image_size(rgbImageSize, depthImageSize), memoryType));
  m_tracker->UpdateInitialPose(m_trackingState.get());
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMShortImage_CPtr SLAMSection::get_input_raw_depth_image() const
{
  return m_inputRawDepthImage;
}

ITMUChar4Image_CPtr SLAMSection::get_input_rgb_image() const
{
  return m_inputRGBImage;
}

const SLAMSection::Scene_Ptr& SLAMSection::get_scene()
{
  return m_scene;
}

Vector2i SLAMSection::get_tracked_image_size(const Vector2i& rgbImageSize, const Vector2i& depthImageSize) const
{
  return m_trackingController->GetTrackedImageSize(rgbImageSize, depthImageSize);
}

const SLAMSection::TrackingState_Ptr& SLAMSection::get_tracking_state()
{
  return m_trackingState;
}

bool SLAMSection::run(SLAMState& state)
{
  if(!m_imageSourceEngine->hasMoreImages()) return false;

  const Raycaster::RenderState_Ptr& liveRenderState = state.get_raycaster()->get_live_render_state();
  const Model::TrackingState_Ptr& trackingState = state.get_model()->get_tracking_state();
  const Model::View_Ptr& view = state.get_model()->get_view();

  // Get the next frame.
  ITMView *newView = view.get();
  m_imageSourceEngine->getImages(m_inputRGBImage.get(), m_inputRawDepthImage.get());
  const bool useBilateralFilter = false;
  m_viewBuilder->UpdateView(&newView, m_inputRGBImage.get(), m_inputRawDepthImage.get(), useBilateralFilter);
  state.get_model()->set_view(newView);

  // Track the camera (we can only do this once we've started reconstructing the model because we need something to track against).
  SE3Pose oldPose(*trackingState->pose_d);
  if(m_fusedFramesCount > 0) m_trackingController->Track(trackingState.get(), view.get());

  // Determine the tracking quality, taking into account the failure mode being used.
  ITMTrackingState::TrackingResult trackerResult = trackingState->trackerResult;
  switch(state.get_model()->get_settings()->behaviourOnFailure)
  {
    case ITMLibSettings::FAILUREMODE_RELOCALISE:
    {
      // Copy the current depth input across to the CPU for use by the relocaliser.
      view->depth->UpdateHostFromDevice();

      // Decide whether or not the relocaliser should consider using this frame as a keyframe.
      bool considerKeyframe = false;
      if(trackerResult == ITMTrackingState::TRACKING_GOOD)
      {
        if(m_keyframeDelay == 0) considerKeyframe = true;
        else --m_keyframeDelay;
      }

      // Process the current depth image using the relocaliser. This attempts to find the nearest keyframe (if any)
      // that is currently in the database, and may add the current frame as a new keyframe if the tracking has been
      // good for some time and the current frame differs sufficiently from the existing keyframes.
      int nearestNeighbour;
      int keyframeID = state.get_relocaliser()->ProcessFrame(view->depth, 1, &nearestNeighbour, NULL, considerKeyframe);

      if(keyframeID >= 0)
      {
        // If the relocaliser added the current frame as a new keyframe, store its pose in the pose database.
        // Note that a new keyframe will only have been added if the tracking quality for this frame was good.
        state.get_pose_database()->storePose(keyframeID, *trackingState->pose_d, 0);
      }
      else if(trackerResult == ITMTrackingState::TRACKING_FAILED && nearestNeighbour != -1)
      {
        // If the tracking failed but a nearest keyframe was found by the relocaliser, reset the pose to that
        // of the keyframe and rerun the tracker for this frame.
        trackingState->pose_d->SetFrom(&state.get_pose_database()->retrievePose(nearestNeighbour).pose);

        const bool resetVisibleList = true;
        m_denseMapper->UpdateVisibleList(view.get(), trackingState.get(), m_scene.get(), liveRenderState.get(), resetVisibleList);
        m_trackingController->Prepare(trackingState.get(), m_scene.get(), view.get(), state.get_raycaster()->get_visualisation_engine().get(), liveRenderState.get());
        m_trackingController->Track(trackingState.get(), view.get());
        trackerResult = trackingState->trackerResult;

        // Set the number of frames for which the tracking quality must be good before the relocaliser can consider
        // adding a new keyframe.
        m_keyframeDelay = 10;
      }

      break;
    }
    case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
    {
      // Since we're not using relocalisation, treat tracking failures like poor tracking,
      // on the basis that it's better to try to keep going than to fail completely.
      if(trackerResult == ITMTrackingState::TRACKING_FAILED) trackerResult = ITMTrackingState::TRACKING_POOR;

      break;
    }
    case ITMLibSettings::FAILUREMODE_IGNORE:
    default:
    {
      // If we're completely ignoring poor or failed tracking, treat the tracking quality as good.
      trackerResult = ITMTrackingState::TRACKING_GOOD;
      break;
    }
  }

  // Decide whether or not fusion should be run.
  bool runFusion = state.get_fusion_enabled();
  if(trackerResult == ITMTrackingState::TRACKING_FAILED ||
     (trackerResult == ITMTrackingState::TRACKING_POOR && m_fusedFramesCount >= m_initialFramesToFuse) ||
     (m_fallibleTracker && m_fallibleTracker->lost_tracking()))
  {
    runFusion = false;
  }

  if(runFusion)
  {
    // Run the fusion process.
    m_denseMapper->ProcessFrame(view.get(), trackingState.get(), m_scene.get(), liveRenderState.get());
    ++m_fusedFramesCount;
  }
  else if(trackerResult != ITMTrackingState::TRACKING_FAILED)
  {
    // If we're not fusing, but the tracking has not completely failed, update the list of visible blocks so that things are kept up to date.
    m_denseMapper->UpdateVisibleList(view.get(), trackingState.get(), m_scene.get(), liveRenderState.get());
  }
  else
  {
    // If the tracking has completely failed, restore the pose from the previous frame.
    *trackingState->pose_d = oldPose;
  }

  // Raycast from the live camera position to prepare for tracking in the next frame.
  m_trackingController->Prepare(trackingState.get(), m_scene.get(), view.get(), state.get_raycaster()->get_visualisation_engine().get(), liveRenderState.get());

  // If the current sub-engine has run out of images, disable fusion.
  if(!m_imageSourceEngine->getCurrentSubengine()->hasMoreImages()) state.set_fusion_enabled(false);

  return true;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

ITMTracker *SLAMSection::make_hybrid_tracker(ITMTracker *primaryTracker, const Settings_Ptr& settings, const Model::Scene_Ptr& scene,
                                          const Vector2i& rgbImageSize, const Vector2i& depthImageSize) const
{
  ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
  compositeTracker->SetTracker(primaryTracker, 0);
  compositeTracker->SetTracker(
    ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().MakeICPTracker(
      rgbImageSize, depthImageSize, settings->deviceType, ORUtils::KeyValueConfig(settings->trackerConfig), m_lowLevelEngine.get(), m_imuCalibrator.get(), scene.get()
    ), 1
  );
  return compositeTracker;
}

void SLAMSection::setup_tracker(const Settings_Ptr& settings, const Model::Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize)
{
  m_fallibleTracker = NULL;

  switch(m_trackerType)
  {
    case TRACKER_RIFT:
    {
#ifdef WITH_OVR
      m_state.m_tracker.reset(make_hybrid_tracker(new RiftTracker, settings, scene, rgbImageSize, depthImageSize));
      break;
#else
      // This should never happen as things stand - we never try to use the Rift tracker if Rift support isn't available.
      throw std::runtime_error("Error: Rift support not currently available. Reconfigure in CMake with the WITH_OVR option set to on.");
#endif
    }
    case TRACKER_ROBUSTVICON:
    {
#ifdef WITH_VICON
      m_fallibleTracker = new RobustViconTracker(m_trackerParams, "kinect", rgbImageSize, depthImageSize, settings, m_lowLevelEngine, scene);
      m_tracker.reset(m_fallibleTracker);
      break;
#else
      // This should never happen as things stand - we never try to use the robust Vicon tracker if Vicon support isn't available.
      throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
    }
    case TRACKER_VICON:
    {
#ifdef WITH_VICON
      m_fallibleTracker = new ViconTracker(m_trackerParams, "kinect");
      m_tracker.reset(make_hybrid_tracker(m_fallibleTracker, settings, scene, rgbImageSize, depthImageSize));
      break;
#else
      // This should never happen as things stand - we never try to use the Vicon tracker if Vicon support isn't available.
      throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
    }
    default: // TRACKER_INFINITAM
    {
      m_imuCalibrator.reset(new ITMIMUCalibrator_iPad);
      m_tracker.reset(ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().Make(
        rgbImageSize, depthImageSize, settings.get(), m_lowLevelEngine.get(), m_imuCalibrator.get(), scene.get()
      ));
    }
  }
}
