/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponent.h"

#include <boost/filesystem.hpp>
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>
namespace bf = boost::filesystem;

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Objects/Camera/ITMCalibIO.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;

#ifdef WITH_GROVE
#include <grove/relocalisation/ScoreRelocaliserFactory.h>
using namespace grove;
#endif

#ifdef WITH_OPENCV
#include <itmx/ocv/OpenCVUtil.h>
#endif
#include <itmx/relocalisation/FernRelocaliser.h>
#include <itmx/relocalisation/ICPRefiningRelocaliser.h>
#include <itmx/remotemapping/RGBDCalibrationMessage.h>
#include <itmx/trackers/TrackerFactory.h>
using namespace itmx;

#include <orx/relocalisation/NullRelocaliser.h>
#include <orx/relocalisation/Relocaliser.h>
using namespace orx;

#include <tvgutil/misc/SettingsContainer.h>
using namespace tvgutil;

#include "segmentation/SegmentationUtil.h"

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
  m_relocaliserTrainingCount(0),
  m_relocaliserTrainingSkip(0),
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
  slamState->set_input_rgb_image(ORUChar4Image_Ptr(new ORUChar4Image(rgbImageSize, true, true)));
  slamState->set_input_raw_depth_image(ORShortImage_Ptr(new ORShortImage(depthImageSize, true, true)));

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

const std::string& SLAMComponent::get_scene_id() const
{
  return m_sceneID;
}

void SLAMComponent::load_models(const std::string& inputDir)
{
  // Reset the scene.
  reset_scene();

  // Load the voxel model. Note that we have to add '/' to the directory in order to force
  // InfiniTAM's loading function to load the files from *inside* the specified folder.
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  slamState->get_voxel_scene()->LoadFromDirectory(inputDir + "/");

  // TODO: If we support surfel model loading at some point in the future, the surfel model should be loaded here as well.

  // Load the relocaliser.
  m_context->get_relocaliser(m_sceneID)->load_from_disk(inputDir);

  // Set up the view to allow the scene to be rendered without any frames needing to be processed.
  // We are aiming to roughly mirror what would happen if we reconstructed the scene frame-by-frame.
  const ORShortImage_Ptr& inputRawDepthImage = slamState->get_input_raw_depth_image();
  const ORUChar4Image_Ptr& inputRGBImage = slamState->get_input_rgb_image();
  const View_Ptr& view = slamState->get_view();

  ITMView *newView = view.get();
  inputRGBImage->Clear();
  inputRawDepthImage->Clear();
  const bool useBilateralFilter = false;
  m_viewBuilder->UpdateView(&newView, inputRGBImage.get(), inputRawDepthImage.get(), useBilateralFilter);
  slamState->set_view(newView);

  // Set the tracking to failed and disable fusion, since we don't know where we are after loading the models.
  slamState->get_tracking_state()->trackerResult = ITMTrackingState::TRACKING_FAILED;
  set_fusion_enabled(false);
}

void SLAMComponent::mirror_pose_of(const std::string& mirrorSceneID)
{
  m_mirrorSceneID = mirrorSceneID;
}

bool SLAMComponent::process_frame()
{
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);

  if(m_imageSourceEngine->hasImagesNow())
  {
    slamState->set_input_status(SLAMState::IS_ACTIVE);
  }
  else
  {
    const SLAMState::InputStatus inputStatus = m_imageSourceEngine->hasMoreImages() ? SLAMState::IS_IDLE : SLAMState::IS_TERMINATED;

    // If finish training is enabled and no more images are expected, let the relocaliser know that no more calls will be made to its train or update functions.
    if(m_finishTrainingEnabled && inputStatus == SLAMState::IS_TERMINATED && slamState->get_input_status() != SLAMState::IS_TERMINATED)
    {
      m_context->get_relocaliser(m_sceneID)->finish_training();
    }

    slamState->set_input_status(inputStatus);

    return false;
  }

  const ORShortImage_Ptr& inputRawDepthImage = slamState->get_input_raw_depth_image();
  const ORUChar4Image_Ptr& inputRGBImage = slamState->get_input_rgb_image();
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
  ORFloatImage_Ptr maskedDepthImage;
  ORUCharImage_CPtr inputMask = m_context->get_slam_state(m_sceneID)->get_input_mask();
  if(inputMask && inputMask->noDims == view->depth->noDims)
  {
    view->depth->UpdateHostFromDevice();
    maskedDepthImage = SegmentationUtil::apply_mask(inputMask, ORFloatImage_CPtr(view->depth, boost::serialization::null_deleter()), -1.0f);
    maskedDepthImage->UpdateDeviceFromHost();
    view->depth->Swap(*maskedDepthImage);
  }

  // Make a note of the current pose in case tracking fails.
  SE3Pose oldPose(*trackingState->pose_d);

  // If we're mirroring the pose of another scene, copy the pose from that scene's tracking state.
  // If not, use our own tracker to estimate the pose.
  if(m_mirrorSceneID != "")
  {
    *trackingState->pose_d = m_context->get_slam_state(m_mirrorSceneID)->get_pose();
    trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
  }
  else
  {
    // Note: When using a normal tracker, it's safe to call this even before we've started fusion (it will be a no-op).
    //       When using a file-based tracker, we *must* call it in order to correctly set the pose for the first frame.
    m_trackingController->Track(trackingState.get(), view.get());
  }

  // If there was an active input mask, restore the original depth image after tracking.
  if(maskedDepthImage) view->depth->Swap(*maskedDepthImage);

  // Determine the tracking quality, taking into account the failure mode being used.
  switch(m_context->get_settings()->behaviourOnFailure)
  {
    case ITMLibSettings::FAILUREMODE_RELOCALISE:
    {
      // Allow the relocaliser to either improve the pose, store a new keyframe or update its model.
      process_relocalisation();
      break;
    }
    case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
    {
      // Since we're not using relocalisation, treat tracking failures like poor tracking,
      // on the basis that it's better to try to keep going than to fail completely.
      if(trackingState->trackerResult == ITMTrackingState::TRACKING_FAILED)
      {
        trackingState->trackerResult = ITMTrackingState::TRACKING_POOR;
      }
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

  // Decide whether or not we need to reset the visible list. This is necessary if we won't be rendering
  // point clouds during tracking, since otherwise space carving won't work.
  const bool resetVisibleList = !m_tracker->requiresPointCloudRendering();

  if(runFusion)
  {
    // Run the fusion process.
    m_denseVoxelMapper->ProcessFrame(view.get(), trackingState.get(), voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
    if(m_mappingMode != MAP_VOXELS_ONLY)
    {
      m_denseSurfelMapper->ProcessFrame(view.get(), trackingState.get(), surfelScene.get(), liveSurfelRenderState.get());
    }

    // If a mapping client is active, use it to send the current frame to the remote mapping server.
    if(m_mappingClient)
    {
      MappingClient::RGBDFrameMessageQueue::PushHandler_Ptr pushHandler = m_mappingClient->begin_push_frame_message();
      boost::optional<RGBDFrameMessage_Ptr&> elt = pushHandler->get();
      if(elt)
      {
        RGBDFrameMessage& msg = **elt;
        msg.set_frame_index(static_cast<int>(m_fusedFramesCount));
        msg.set_pose(*trackingState->pose_d);
        msg.set_rgb_image(inputRGBImage);
        msg.set_depth_image(inputRawDepthImage);
      }
    }

    ++m_fusedFramesCount;
  }
  else if(trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
  {
    // If we're not fusing, but the tracking has not completely failed, update the list of visible blocks so that things are kept up to date.
    m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(), voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
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
  m_relocaliserTrainingCount = 0;

  // Reset some variables to their initial values.
  m_fusedFramesCount = 0;
  m_fusionEnabled = true;
}

void SLAMComponent::save_models(const std::string& outputDir) const
{
  // Make sure that the output directory exists.
  bf::create_directories(outputDir);

  // Save the camera calibration.
  SLAMState_CPtr slamState = m_context->get_slam_state(m_sceneID);
  const std::string calibFilename = outputDir + "/calib.txt";
  writeRGBDCalib(calibFilename.c_str(), slamState->get_view()->calib);

  // Save relevant settings to a configuration file.
  Settings_CPtr settings = m_context->get_settings();
  const ITMSceneParams& sceneParams = settings->sceneParams;
  const std::string configFilename = outputDir + "/settings.ini";
  {
    std::ofstream fs(configFilename.c_str());
    fs << "[SceneParams]\n";
    fs << "mu = " << sceneParams.mu << '\n';
    fs << "viewFrustum_max = " << sceneParams.viewFrustum_max << '\n';
    fs << "viewFrustum_min = " << sceneParams.viewFrustum_min << '\n';
    fs << "voxelSize = " << sceneParams.voxelSize << '\n';
  }

  // Save the voxel model. Note that we have to add '/' to the directory in order to force
  // InfiniTAM's saving function to save the files *inside* the specified folder.
  slamState->get_voxel_scene()->SaveToDirectory(outputDir + "/");

  // TODO: If we support surfel model saving at some point in the future, the surfel model should be saved here as well.

  // Save the relocaliser.
  m_context->get_relocaliser(m_sceneID)->save_to_disk(outputDir);
}

void SLAMComponent::set_detect_fiducials(bool detectFiducials)
{
  m_detectFiducials = detectFiducials;
}

void SLAMComponent::set_fusion_enabled(bool fusionEnabled)
{
  m_fusionEnabled = fusionEnabled;
}

void SLAMComponent::set_mapping_client(const MappingClient_Ptr& mappingClient)
{
  m_mappingClient = mappingClient;

  // If we're using a mapping client, send an initial calibration message across to the server.
  if(m_mappingClient)
  {
    SLAMState_CPtr slamState = m_context->get_slam_state(m_sceneID);

    RGBDCalibrationMessage calibMsg;
    calibMsg.set_calib(m_imageSourceEngine->getCalib());

    // TODO: Allow these to be configured from the command line.
#ifdef WITH_OPENCV
    calibMsg.set_depth_compression_type(DEPTH_COMPRESSION_PNG);
    calibMsg.set_rgb_compression_type(RGB_COMPRESSION_JPG);
#else
    calibMsg.set_depth_compression_type(DEPTH_COMPRESSION_NONE);
    calibMsg.set_rgb_compression_type(RGB_COMPRESSION_NONE);
#endif

    std::cout << "Sending calibration message" << std::endl;
    m_mappingClient->send_calibration_message(calibMsg);
  }
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
  const Relocaliser_Ptr& relocaliser = m_context->get_relocaliser(m_sceneID);
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();
  const View_Ptr& view = slamState->get_view();
  const Vector4f& depthIntrinsics = view->calib.intrinsics_d.projectionParamsSimple.all;

  // Save the current pose in case we need to restore it later.
  const SE3Pose oldPose(*trackingState->pose_d);

  // Decide whether or not to perform training in this frame. We train iff either of the following is true:
  // - Relocalising every frame is enabled
  // - The tracking succeeded and the current frame is not one we should skip
  const bool performTraining =
    m_relocaliseEveryFrame ||
    (
      trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD &&
      (m_relocaliserTrainingSkip == 0 || (m_relocaliserTrainingCount++ % m_relocaliserTrainingSkip == 0))
    );

  // If we're not training in this frame, allow the relocaliser to perform any necessary internal bookkeeping.
  // Note that we prevent training and bookkeeping from both running in the same frame for performance reasons.
  if(!performTraining)
  {
    relocaliser->update();
  }

  // Relocalise if either (a) the tracking has failed, or (b) we're forcibly relocalising every frame for evaluation purposes.
  const bool performRelocalisation = m_relocaliseEveryFrame || trackingState->trackerResult == ITMTrackingState::TRACKING_FAILED;
  if(performRelocalisation)
  {
    std::vector<Relocaliser::Result> relocalisationResults = relocaliser->relocalise(view->rgb, view->depth, depthIntrinsics);

    if(!relocalisationResults.empty())
    {
      const Relocaliser::Result& bestRelocalisationResult = relocalisationResults[0];
      trackingState->pose_d->SetFrom(&bestRelocalisationResult.pose);
      trackingState->trackerResult = bestRelocalisationResult.quality == Relocaliser::RELOCALISATION_GOOD ? ITMTrackingState::TRACKING_GOOD : ITMTrackingState::TRACKING_POOR;
    }
  }

  // Train the relocaliser if necessary.
  if(performTraining)
  {
    relocaliser->train(view->rgb, view->depth, depthIntrinsics, oldPose);
  }

  // If we're relocalising and training every frame for evaluation purposes, restore the original pose. The assumption
  // is that if we're doing this, it's because we're using a ground truth trajectory from disk, and so we're only
  // interested in whether the relocaliser would have succeeded, not in keeping the poses it produces.
  if(m_relocaliseEveryFrame)
  {
    trackingState->pose_d->SetFrom(&oldPose);
    trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
  }
}

void SLAMComponent::setup_relocaliser()
{
  const Vector2i depthImageSize = m_imageSourceEngine->getDepthImageSize();
  const Vector2i rgbImageSize = m_imageSourceEngine->getRGBImageSize();
  const Settings_CPtr& settings = m_context->get_settings();
  const SpaintVoxelScene_Ptr& voxelScene = m_context->get_slam_state(m_sceneID)->get_voxel_scene();

  // Look up the non-relocaliser-specific settings, such as the type of relocaliser to construct.
  // Note that the relocaliser type is a primary setting, so is not in the SLAMComponent namespace.
  m_relocaliserType = settings->get_first_value<std::string>("relocaliserType");

  static const std::string settingsNamespace = "SLAMComponent.";
  m_finishTrainingEnabled = settings->get_first_value<bool>(settingsNamespace + "finishTrainingEnabled", false);
  m_relocaliseEveryFrame = settings->get_first_value<bool>(settingsNamespace + "relocaliseEveryFrame", false);
  m_relocaliserTrainingSkip = settings->get_first_value<size_t>(settingsNamespace + "relocaliserTrainingSkip", 0);

#ifndef WITH_GROVE
  // If the user is trying to use the Grove relocaliser and it has not been built, fall back to the ferns relocaliser and issue a warning.
  if(m_relocaliserType == "forest")
  {
    m_relocaliserType = "ferns";
    std::cerr << "Warning: Cannot use a Grove relocaliser because BUILD_GROVE is disabled in CMake. Falling back to random ferns.\n";
  }
#endif

  // Construct a relocaliser of the specified type.
  Relocaliser_Ptr innerRelocaliser;
  if(m_relocaliserType == "forest")
  {
#ifdef WITH_GROVE
    // If we're trying to set up a forest-based relocaliser, determine the path to the file containing the forest.
    const std::string defaultRelocalisationForestPath = (bf::path(m_context->get_resources_dir()) / "DefaultRelocalisationForest.rf").string();
    m_relocaliserForestPath = settings->get_first_value<std::string>(settingsNamespace + "relocalisationForestPath", defaultRelocalisationForestPath);
    std::cout << "Loading relocalisation forest from: " << m_relocaliserForestPath << '\n';

    // Load the relocaliser from the specified file.
    innerRelocaliser = ScoreRelocaliserFactory::make_score_relocaliser(m_relocaliserForestPath, settings, settings->deviceType);
#endif
  }
  else if(m_relocaliserType == "ferns")
  {
    innerRelocaliser.reset(new FernRelocaliser(
      depthImageSize,
      settings->sceneParams.viewFrustum_min,
      settings->sceneParams.viewFrustum_max,
      FernRelocaliser::get_default_harvesting_threshold(),
      FernRelocaliser::get_default_num_ferns(),
      FernRelocaliser::get_default_num_decisions_per_fern(),
      m_relocaliseEveryFrame ? FernRelocaliser::ALWAYS_TRY_ADD : FernRelocaliser::DELAY_AFTER_RELOCALISATION
    ));
  }
  else if(m_relocaliserType == "none")
  {
    innerRelocaliser.reset(new NullRelocaliser);
  }
  else throw std::invalid_argument("Invalid relocaliser type: " + m_relocaliserType);

  // Now decorate this relocaliser with one that uses an ICP tracker to refine the results.
  std::string trackerConfig = "<tracker type='infinitam'>";
  std::string trackerParams = settings->get_first_value<std::string>(settingsNamespace + "refinementTrackerParams", "");
  if(trackerParams != "") trackerConfig += "<params>" + trackerParams + "</params>";
  trackerConfig += "</tracker>";

  const bool trackSurfels = false;
  FallibleTracker *dummy;
  Tracker_Ptr tracker = TrackerFactory::make_tracker_from_string(trackerConfig, trackSurfels, rgbImageSize, depthImageSize, m_lowLevelEngine, m_imuCalibrator, settings, dummy);

  m_context->get_relocaliser(m_sceneID).reset(new ICPRefiningRelocaliser<SpaintVoxel,ITMVoxelIndex>(
    innerRelocaliser, tracker, rgbImageSize, depthImageSize, m_imageSourceEngine->getCalib(), voxelScene, m_denseVoxelMapper, settings
  ));
}

void SLAMComponent::setup_tracker()
{
  const MappingServer_Ptr& mappingServer = m_context->get_mapping_server();
  const Settings_CPtr& settings = m_context->get_settings();
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const Vector2i& depthImageSize = slamState->get_depth_image_size();
  const Vector2i& rgbImageSize = slamState->get_rgb_image_size();

  m_imuCalibrator.reset(new ITMIMUCalibrator_iPad);
  m_tracker = TrackerFactory::make_tracker_from_string(
    m_trackerConfig, m_trackingMode == TRACK_SURFELS, rgbImageSize, depthImageSize, m_lowLevelEngine, m_imuCalibrator, settings, m_fallibleTracker, mappingServer
  );
}

}
