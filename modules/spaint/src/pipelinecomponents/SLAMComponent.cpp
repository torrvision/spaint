/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponent.h"

#include <boost/filesystem.hpp>
namespace bf = boost::filesystem;
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;

#include <grove/relocalisation/ScoreRelocaliserFactory.h>
using namespace grove;

#include <itmx/relocalisation/ICPRefiningRelocaliser.h>
#include <itmx/relocalisation/RelocaliserFactory.h>
using namespace itmx;

#include <tvgutil/misc/SettingsContainer.h>
using namespace tvgutil;

#include "segmentation/SegmentationUtil.h"
#include "trackers/TrackerFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SLAMComponent::SLAMComponent(const SLAMContext_Ptr& context, const std::string& sceneID, const ImageSourceEngine_Ptr& imageSourceEngine,
                             const std::string& trackerConfig, MappingMode mappingMode, TrackingMode trackingMode,
                             const FiducialDetector_CPtr& fiducialDetector, bool detectFiducials)
: m_context(context),
  m_detectFiducials(detectFiducials),
  m_fallibleTracker(NULL),
  m_fiducialDetector(fiducialDetector),
  m_imageSourceEngine(imageSourceEngine),
  m_initialFramesToFuse(50), // FIXME: This value should be passed in rather than hard-coded.
  m_mappingMode(mappingMode),
  m_sceneID(sceneID),
  m_trackerConfig(trackerConfig),
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

  // Set up the relocaliser.
  setup_relocaliser();

  // Set up the live render states.
  slamState->set_live_voxel_render_state(VoxelRenderState_Ptr(ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(trackedImageSize, voxelScene->sceneParams, memoryType)));
  if(mappingMode != MAP_VOXELS_ONLY)
  {
    slamState->set_live_surfel_render_state(SurfelRenderState_Ptr(new ITMSurfelRenderState(trackedImageSize, settings->surfelSceneParams.supersamplingFactor)));
  }

  // Set up the scene.
  reset_scene();
}

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
  switch(m_context->get_settings()->behaviourOnFailure)
  {
    case ITMLibSettings::FAILUREMODE_RELOCALISE:
    {
      // Allow the relocaliser to either improve the pose or store a new keyframe, update its model, etc...
      process_relocalisation();
      break;
    }
    case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
    {
      // Since we're not using relocalisation, treat tracking failures like poor tracking,
      // on the basis that it's better to try to keep going than to fail completely.
      if(trackingState->trackerResult == ITMTrackingState::TRACKING_FAILED)
        trackingState->trackerResult = ITMTrackingState::TRACKING_POOR;

      break;
    }
    case ITMLibSettings::FAILUREMODE_IGNORE:
    default:
    {
      // If we're completely ignoring poor or failed tracking, treat the tracking quality as good.
      trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
      break;
    }
  }

  // Decide whether or not fusion should be run.
  bool runFusion = m_fusionEnabled;
  if(trackingState->trackerResult == ITMTrackingState::TRACKING_FAILED ||
     (trackingState->trackerResult == ITMTrackingState::TRACKING_POOR && m_fusedFramesCount >= m_initialFramesToFuse) ||
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
  else if(trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
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
  if(m_fiducialDetector && m_detectFiducials && trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD)
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

  // Reset the relocaliser.
  m_context->get_relocaliser(m_sceneID)->reset();

  // Reset some variables to their initial values.
  m_fusedFramesCount = 0;
  m_fusionEnabled = true;
}

void SLAMComponent::set_detect_fiducials(bool detectFiducials)
{
  m_detectFiducials = detectFiducials;
}

void SLAMComponent::set_fusion_enabled(bool fusionEnabled)
{
  m_fusionEnabled = fusionEnabled;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

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

void SLAMComponent::process_relocalisation()
{
  const Relocaliser_Ptr &relocaliser = m_context->get_relocaliser(m_sceneID);
  const SLAMState_Ptr &slamState = m_context->get_slam_state(m_sceneID);
  const TrackingState_Ptr &trackingState = slamState->get_tracking_state();
  const View_Ptr &view = slamState->get_view();

  const Vector4f depthIntrinsics = view->calib.intrinsics_d.projectionParamsSimple.all;
  const ITMFloatImage *inputDepthImage = view->depth;
  const ITMUChar4Image *inputRGBImage = view->rgb;

  const bool performRelocalization = m_relocaliseEveryFrame || trackingState->trackerResult == ITMTrackingState::TRACKING_FAILED;
  const bool performLearning = m_relocaliseEveryFrame || trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD;

  // Save the tracked pose just in case we need to set it at the end (if m_relocaliseEveryFrame is true).
  const SE3Pose trackedPose(*trackingState->pose_d);

  if (m_relocaliserUpdateEveryFrame && !performLearning)
  {
    relocaliser->update();
  }

  if (performRelocalization)
  {
    boost::optional<Relocaliser::Result> relocalisationResult =
        relocaliser->relocalise(inputRGBImage, inputDepthImage, depthIntrinsics);

    if (relocalisationResult)
    {
      trackingState->pose_d->SetFrom(&(relocalisationResult->pose));
      trackingState->trackerResult = relocalisationResult->quality == Relocaliser::RELOCALISATION_GOOD
                                     ? ITMTrackingState::TRACKING_GOOD
                                     : ITMTrackingState::TRACKING_POOR;
    }
  }

  if (performLearning)
  {
    relocaliser->train(inputRGBImage, inputDepthImage, depthIntrinsics, trackedPose);
  }

  if (m_relocaliseEveryFrame)
  {
    // Restore tracked pose.
    trackingState->pose_d->SetFrom(&trackedPose);
    // The assumption is that we are using the ground truth trajectory, so the tracker always succeeds.
    trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
  }
}

void SLAMComponent::setup_relocaliser()
{
  const Settings_CPtr& settings = m_context->get_settings();
  const static std::string settingsNamespace = "SLAMComponent.";

  m_relocaliseEveryFrame = settings->get_first_value<bool>(settingsNamespace +
                                                          "relocaliseEveryFrame", false);

  m_relocaliserType = settings->get_first_value<std::string>(settingsNamespace +
                                                            "relocaliserType", "forest");

  m_relocaliserUpdateEveryFrame = settings->get_first_value<bool>(settingsNamespace +
                                                                 "updateRelocaliserEveryFrame", true);

  // Useful variables.
  const Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  const Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  const SpaintVoxelScene_Ptr& voxelScene = m_context->get_slam_state(m_sceneID)->get_voxel_scene();

  // A pointer to the actual relocaliser that will be nested in the refining relocaliser.
  Relocaliser_Ptr nestedRelocaliser;
  if (m_relocaliserType == "forest")
  {
    const std::string defaultRelocalisationForestPath = (bf::path(m_context->get_resources_dir()) /
                                                         "DefaultRelocalizationForest.rf").string();

    m_relocaliserForestPath = settings->get_first_value<std::string>(settingsNamespace +
                                                                     "relocalisationForestPath", defaultRelocalisationForestPath);
    std::cout << "Loading relocalization forest from: " << m_relocaliserForestPath << '\n';

    nestedRelocaliser = ScoreRelocaliserFactory::make_score_relocaliser(settings->deviceType, settings, m_relocaliserForestPath);
    //  nestedRelocaliser = ScoreRelocaliserFactory::make_score_relocaliser(ITMLibSettings::DEVICE_CPU, m_relocalisationForestPath);
  }
  else if (m_relocaliserType == "ferns")
  {
    if (m_relocaliseEveryFrame)
    {
      // Need to force the policy allowing the learning of new keyframes after relocalisation.
      nestedRelocaliser = RelocaliserFactory::make_custom_fern_relocaliser(depthImageSize,
                                                                           settings->sceneParams.viewFrustum_min,
                                                                           settings->sceneParams.viewFrustum_max,
                                                                           FernRelocaliser::get_default_harvesting_threshold(),
                                                                           FernRelocaliser::get_default_num_ferns(),
                                                                           FernRelocaliser::get_default_num_decisions_per_fern(),
                                                                           FernRelocaliser::ALWAYS_TRY_ADD);
    }
    else
    {
      nestedRelocaliser = RelocaliserFactory::make_default_fern_relocaliser(depthImageSize,
                                                                            settings->sceneParams.viewFrustum_min,
                                                                            settings->sceneParams.viewFrustum_max);
    }
  }
  else
  {
    throw std::invalid_argument("Invalid relocaliser type: " + m_relocaliserType);
  }

  // Refinement ICP tracker
  m_relocaliserRefinementTrackerParams = settings->get_first_value<std::string>(settingsNamespace + "refinementTrackerParams",
                                                                                "type=extended,levels=rrbb,minstep=1e-4,"
                                                                                "outlierSpaceC=0.1,outlierSpaceF=0.004,"
                                                                                "numiterC=20,numiterF=20,tukeyCutOff=8,"
                                                                                "framesToSkip=20,framesToWeight=50,failureDec=20.0");

  // Set up the refining relocaliser.
  m_context->get_relocaliser(m_sceneID).reset(new ICPRefiningRelocaliser<SpaintVoxel,ITMVoxelIndex>(
    nestedRelocaliser, m_imageSourceEngine->getCalib(), rgbImageSize, depthImageSize,
    voxelScene, settings, m_relocaliserRefinementTrackerParams
  ));
}

void SLAMComponent::setup_tracker()
{
  const Settings_CPtr& settings = m_context->get_settings();
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const Vector2i& depthImageSize = slamState->get_depth_image_size();
  const Vector2i& rgbImageSize = slamState->get_rgb_image_size();

  m_imuCalibrator.reset(new ITMIMUCalibrator_iPad);
  m_tracker = TrackerFactory::make_tracker_from_string(m_trackerConfig, m_trackingMode == TRACK_SURFELS, rgbImageSize, depthImageSize, m_lowLevelEngine, m_imuCalibrator, settings, m_fallibleTracker);
}

}
