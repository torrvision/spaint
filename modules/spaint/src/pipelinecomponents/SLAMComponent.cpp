/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponent.h"

#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>
using namespace FernRelocLib;
using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;

#include "segmentation/SegmentationUtil.h"

#ifdef WITH_OVR
#include "trackers/RiftTracker.h"
#endif

#ifdef WITH_VICON
#include "trackers/RobustViconTracker.h"
#include "trackers/ViconTracker.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

SLAMComponent::SLAMComponent(const SLAMContext_Ptr& context, const std::string& sceneID, const ImageSourceEngine_Ptr& imageSourceEngine,
                             TrackerType trackerType, const std::vector<std::string>& trackerParams, MappingMode mappingMode, TrackingMode trackingMode,
                             const FiducialDetector_CPtr& fiducialDetector, bool detectFiducials)
: m_context(context),
  m_detectFiducials(detectFiducials),
  m_fiducialDetector(fiducialDetector),
  m_imageSourceEngine(imageSourceEngine),
  m_initialFramesToFuse(50), // FIXME: This value should be passed in rather than hard-coded.
  m_mappingMode(mappingMode),
  m_sceneID(sceneID),
  m_trackerParams(trackerParams),
  m_trackerType(trackerType),
  m_trackingMode(trackingMode)
{
  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the RGB and raw depth images into which input is to be read each frame.
  const SLAMState_Ptr& slamState = context->get_slam_state(sceneID);
  slamState->set_input_rgb_image(ITMUChar4Image_Ptr(new ITMUChar4Image(rgbImageSize, true, true)));
  slamState->set_input_raw_depth_image(ITMShortImage_Ptr(new ITMShortImage(depthImageSize, true, true)));

  // Set up the low-level engine.
  const Settings_CPtr& settings = context->get_settings();
  m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType));

  // Set up the view builder.
  m_viewBuilder.reset(ITMViewBuilderFactory::MakeViewBuilder(m_imageSourceEngine->getCalib(), settings->deviceType));

  // Set up the scenes.
  MemoryDeviceType memoryType = settings->GetMemoryType();
  slamState->set_voxel_scene(SpaintVoxelScene_Ptr(new SpaintVoxelScene(&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType)));
  if(mappingMode != MAP_VOXELS_ONLY)
  {
    slamState->set_surfel_scene(SpaintSurfelScene_Ptr(new SpaintSurfelScene(&settings->surfelSceneParams, memoryType)));
  }

  // Set up the dense mappers.
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();
  m_denseVoxelMapper.reset(new ITMDenseMapper<SpaintVoxel,ITMVoxelIndex>(settings.get()));
  if(mappingMode != MAP_VOXELS_ONLY)
  {
    m_denseSurfelMapper.reset(new ITMDenseSurfelMapper<SpaintSurfel>(depthImageSize, settings->deviceType));
  }

  // Set up the tracker and the tracking controller.
  setup_tracker();
  m_trackingController.reset(new ITMTrackingController(m_tracker.get(), settings.get()));
  const Vector2i trackedImageSize = m_trackingController->GetTrackedImageSize(rgbImageSize, depthImageSize);
  slamState->set_tracking_state(TrackingState_Ptr(new ITMTrackingState(trackedImageSize, memoryType)));
  m_tracker->UpdateInitialPose(slamState->get_tracking_state().get());

  // Set up the live render states.
  slamState->set_live_voxel_render_state(VoxelRenderState_Ptr(ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(trackedImageSize, voxelScene->sceneParams, memoryType)));
  if(mappingMode != MAP_VOXELS_ONLY)
  {
    slamState->set_live_surfel_render_state(SurfelRenderState_Ptr(new ITMSurfelRenderState(trackedImageSize, settings->surfelSceneParams.supersamplingFactor)));
  }

  // Set up the scene.
  reset_scene();
}

//#################### DESTRUCTOR ####################
SLAMComponent::~SLAMComponent() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool SLAMComponent::get_fusion_enabled() const
{
  return m_fusionEnabled;
}

void SLAMComponent::mirror_pose_of(const std::string& mirrorSceneID)
{
  m_mirrorSceneID = mirrorSceneID;
}

bool SLAMComponent::process_frame()
{
  if(!m_imageSourceEngine->hasMoreImages()) return false;

  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const ITMShortImage_Ptr& inputRawDepthImage = slamState->get_input_raw_depth_image();
  const ITMUChar4Image_Ptr& inputRGBImage = slamState->get_input_rgb_image();
  const SurfelRenderState_Ptr& liveSurfelRenderState = slamState->get_live_surfel_render_state();
  const VoxelRenderState_Ptr& liveVoxelRenderState = slamState->get_live_voxel_render_state();
  const SpaintSurfelScene_Ptr& surfelScene = slamState->get_surfel_scene();
  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();
  const View_Ptr& view = slamState->get_view();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

  // Get the next frame.
  ITMView *newView = view.get();
  m_imageSourceEngine->getImages(inputRGBImage.get(), inputRawDepthImage.get());
  const bool useBilateralFilter = m_trackingMode == TRACK_SURFELS;
  m_viewBuilder->UpdateView(&newView, inputRGBImage.get(), inputRawDepthImage.get(), useBilateralFilter);
  slamState->set_view(newView);

  // If there's an active input mask of the right size, apply it to the depth image.
  ITMFloatImage_Ptr maskedDepthImage;
  ITMUCharImage_CPtr inputMask = m_context->get_slam_state(m_sceneID)->get_input_mask();
  if(inputMask && inputMask->noDims == view->depth->noDims)
  {
    view->depth->UpdateHostFromDevice();
    maskedDepthImage = SegmentationUtil::apply_mask(inputMask, ITMFloatImage_CPtr(view->depth, boost::serialization::null_deleter()), -1.0f);
    maskedDepthImage->UpdateDeviceFromHost();
    view->depth->Swap(*maskedDepthImage);
  }

  // Make a note of the current pose in case tracking fails.
  SE3Pose oldPose(*trackingState->pose_d);

  // If we're mirroring the pose of another scene, copy the pose from that scene's tracking state. If not, use our own tracker
  // to estimate the pose (we can only do this once we've started reconstruction because we need something to track against).
  if(m_mirrorSceneID != "")
  {
    *trackingState->pose_d = m_context->get_slam_state(m_mirrorSceneID)->get_pose();
    trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
  }
  else // if(m_fusedFramesCount > 0)
  {
    // The tracking controller knows how to handle the case where we don't have any fused frame
    // (i.e. when we use a file-based tracker).
    m_trackingController->Track(trackingState.get(), view.get());
  }

  // If there was an active input mask, restore the original depth image after tracking.
  if(maskedDepthImage) view->depth->Swap(*maskedDepthImage);

  // Determine the tracking quality, taking into account the failure mode being used.
  ITMTrackingState::TrackingResult trackerResult = trackingState->trackerResult;
  switch(m_context->get_settings()->behaviourOnFailure)
  {
    case ITMLibSettings::FAILUREMODE_RELOCALISE:
    {
      // Allow the relocaliser to either improve the pose or store a new keyframe, update its model, etc...
      trackerResult = process_relocalisation(trackerResult);
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
  bool runFusion = m_fusionEnabled;
  if(trackerResult == ITMTrackingState::TRACKING_FAILED ||
     (trackerResult == ITMTrackingState::TRACKING_POOR && m_fusedFramesCount >= m_initialFramesToFuse) ||
     (m_fallibleTracker && m_fallibleTracker->lost_tracking()))
  {
    runFusion = false;
  }

  if(runFusion)
  {
    // Run the fusion process.
    m_denseVoxelMapper->ProcessFrame(view.get(), trackingState.get(), voxelScene.get(), liveVoxelRenderState.get());
    if(m_mappingMode != MAP_VOXELS_ONLY)
    {
      m_denseSurfelMapper->ProcessFrame(view.get(), trackingState.get(), surfelScene.get(), liveSurfelRenderState.get());
    }

    ++m_fusedFramesCount;
  }
  else if(trackerResult != ITMTrackingState::TRACKING_FAILED)
  {
    // If we're not fusing, but the tracking has not completely failed, update the list of visible blocks so that things are kept up to date.
    m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(), voxelScene.get(), liveVoxelRenderState.get());
  }
  else
  {
    // If the tracking has completely failed, restore the pose from the previous frame.
    *trackingState->pose_d = oldPose;
  }

  // Render from the live camera position to prepare for tracking in the next frame.
  prepare_for_tracking(m_trackingMode);

  // If we're using surfel mapping, render a supersampled index image to use when finding surfel correspondences in the next frame.
  if(m_mappingMode != MAP_VOXELS_ONLY)
  {
    m_context->get_surfel_visualisation_engine()->FindSurfaceSuper(surfelScene.get(), trackingState->pose_d, &view->calib.intrinsics_d, USR_RENDER, liveSurfelRenderState.get());
  }

  // If we're using a composite image source engine and the current sub-engine has run out of images, disable fusion.
  CompositeImageSourceEngine_CPtr compositeImageSourceEngine = boost::dynamic_pointer_cast<const CompositeImageSourceEngine>(m_imageSourceEngine);
  if(compositeImageSourceEngine && !compositeImageSourceEngine->getCurrentSubengine()->hasMoreImages()) m_fusionEnabled = false;

  // If we're using a fiducial detector and the user wants to detect fiducials and the tracking is good, try to detect fiducial markers
  // in the current view of the scene and update the current set of fiducials that we're maintaining accordingly.
  if(m_fiducialDetector && m_detectFiducials && trackerResult == ITMTrackingState::TRACKING_GOOD)
  {
    slamState->update_fiducials(m_fiducialDetector->detect_fiducials(view, *trackingState->pose_d, liveVoxelRenderState, FiducialDetector::PEM_RAYCAST));
  }

  return true;
}

void SLAMComponent::reset_scene()
{
  // Reset the scene.
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  m_denseVoxelMapper->ResetScene(slamState->get_voxel_scene().get());
  if(m_mappingMode != MAP_VOXELS_ONLY)
  {
    slamState->get_surfel_scene()->Reset();
  }

  // Reset the tracking state.
  slamState->get_tracking_state()->Reset();

  // Reset the pose database and the relocaliser.
  m_poseDatabase.reset(new PoseDatabase);

  const Vector2i& depthImageSize = slamState->get_depth_image_size();
  const float harvestingThreshold = 0.2f;
  const int numFerns = 500;
  const int numDecisionsPerFern = 4;
  const Settings_CPtr& settings = m_context->get_settings();
  m_relocaliser.reset(new Relocaliser<float>(
    depthImageSize,
    Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max),
    harvestingThreshold, numFerns, numDecisionsPerFern
  ));

  // Reset some variables to their initial values.
  m_fusedFramesCount = 0;
  m_fusionEnabled = true;
  m_keyframeDelay = 0;
}

void SLAMComponent::set_detect_fiducials(bool detectFiducials)
{
  m_detectFiducials = detectFiducials;
}

void SLAMComponent::set_fusion_enabled(bool fusionEnabled)
{
  m_fusionEnabled = fusionEnabled;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void SLAMComponent::prepare_for_tracking(TrackingMode trackingMode)
{
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();
  const View_Ptr& view = slamState->get_view();

  switch(trackingMode)
  {
    case TRACK_SURFELS:
    {
      const SpaintSurfelScene_Ptr& surfelScene = slamState->get_surfel_scene();
      const SurfelRenderState_Ptr& liveSurfelRenderState = slamState->get_live_surfel_render_state();
      m_trackingController->Prepare(trackingState.get(), surfelScene.get(), view.get(), m_context->get_surfel_visualisation_engine().get(), liveSurfelRenderState.get());
      break;
    }
    case TRACK_VOXELS:
    default:
    {
      const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();
      const VoxelRenderState_Ptr& liveVoxelRenderState = slamState->get_live_voxel_render_state();
      m_trackingController->Prepare(trackingState.get(), voxelScene.get(), view.get(), m_context->get_voxel_visualisation_engine().get(), liveVoxelRenderState.get());
      break;
    }
  }
}

SLAMComponent::TrackingResult SLAMComponent::process_relocalisation(TrackingResult trackerResult)
{
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const VoxelRenderState_Ptr& liveVoxelRenderState = slamState->get_live_voxel_render_state();
  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();
  const View_Ptr& view = slamState->get_view();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

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
  bool keyframeAdded = m_relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &nearestNeighbour, NULL, considerKeyframe);

  // If no keyframe was added and the tracking failed, but a nearest keyframe was found by the relocaliser, reset
  // the pose to that of the keyframe and rerun the tracker for this frame.
  if(!keyframeAdded && trackerResult == ITMTrackingState::TRACKING_FAILED && nearestNeighbour != -1)
  {
    trackingState->pose_d->SetFrom(&m_relocaliser->RetrievePose(nearestNeighbour).pose);

    const bool resetVisibleList = true;
    m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(), voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
    prepare_for_tracking(TRACK_VOXELS);
    m_trackingController->Track(trackingState.get(), view.get());
    trackerResult = trackingState->trackerResult;

    // Set the number of frames for which the tracking quality must be good before the relocaliser can consider
    // adding a new keyframe.
    m_keyframeDelay = 10;
  }

  return trackerResult;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SLAMComponent::setup_tracker()
{
  const Settings_CPtr& settings = m_context->get_settings();
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const Vector2i& depthImageSize = slamState->get_depth_image_size();
  const Vector2i& rgbImageSize = slamState->get_rgb_image_size();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();
  m_fallibleTracker = NULL;

  // Setup a composite tracker that will be assigned to m_tracker.
  boost::shared_ptr<ITMCompositeTracker> compositeTracker(new ITMCompositeTracker(
      m_trackerParams.size(),
      m_trackerType == TRACKER_INFINITAM_NO_REFINE ? ITMCompositeTracker::POLICY_STOP_ON_FIRST_SUCCESS : ITMCompositeTracker::POLICY_REFINE
  ));

  size_t infinitamFirstTrackerIdx = 0;

  switch(m_trackerType)
  {
    case TRACKER_RIFT:
    {
#ifdef WITH_OVR
      compositeTracker->SetTracker(new RiftTracker, 0);
      infinitamFirstTrackerIdx = 1;
      break;
#else
      // This should never happen as things stand - we never try to use the Rift tracker if Rift support isn't available.
      throw std::runtime_error("Error: Rift support not currently available. Reconfigure in CMake with the WITH_OVR option set to on.");
#endif
    }
    case TRACKER_ROBUSTVICON:
    {
#ifdef WITH_VICON
      m_fallibleTracker = new RobustViconTracker(m_trackerParams[0], "kinect", rgbImageSize, depthImageSize, settings, m_lowLevelEngine, voxelScene);
      compositeTracker->SetTracker(m_fallibleTracker, 0);
      infinitamFirstTrackerIdx = 1;
      break;
#else
      // This should never happen as things stand - we never try to use the robust Vicon tracker if Vicon support isn't available.
      throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
    }
    case TRACKER_VICON:
    {
#ifdef WITH_VICON
      m_fallibleTracker = new ViconTracker(m_trackerParams[0], "kinect");
      compositeTracker->SetTracker(m_fallibleTracker, 0);
      infinitamFirstTrackerIdx = 1;
      break;
#else
      // This should never happen as things stand - we never try to use the Vicon tracker if Vicon support isn't available.
      throw std::runtime_error("Error: Vicon support not currently available. Reconfigure in CMake with the WITH_VICON option set to on.");
#endif
    }
    default:
    {
      m_imuCalibrator.reset(new ITMIMUCalibrator_iPad);
    }
  }

  for(size_t i = infinitamFirstTrackerIdx; i < m_trackerParams.size(); ++i)
  {
    compositeTracker->SetTracker(ITMTrackerFactory::Instance().Make(
      m_trackerParams[i].c_str(), rgbImageSize, depthImageSize, settings.get(), m_lowLevelEngine.get(), m_imuCalibrator.get(), voxelScene->sceneParams
    ), i);
  }

  m_tracker = compositeTracker;
}

}
