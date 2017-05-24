/**
 * itmx: ICPRefiningRelocaliser.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ICPRefiningRelocaliser.h"

#include <iostream>
#include <stdexcept>

#include <boost/filesystem.hpp>

#include <cuda_runtime_api.h>

#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>

#include <ORUtils/PlatformIndependence.h>

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/misc/SettingsContainer.h>
#include <tvgutil/timing/TimeUtil.h>

#include "../persistence/PosePersister.h"

namespace fs = boost::filesystem;
using namespace ITMLib;
using namespace tvgutil;

namespace itmx {

//#################### CONSTRUCTORS ####################

template <typename VoxelType, typename IndexType>
ICPRefiningRelocaliser<VoxelType, IndexType>::ICPRefiningRelocaliser(const Relocaliser_Ptr &relocaliser,
                                                                     const ITMLib::ITMRGBDCalib &calibration,
                                                                     const Vector2i imgSize_rgb,
                                                                     const Vector2i imgsize_d,
                                                                     const Scene_Ptr &scene,
                                                                     const Settings_CPtr &settings,
                                                                     const std::string &trackerConfig)
  : RefiningRelocaliser(relocaliser)
  , m_scene(scene)
  , m_settings(settings)
  , m_timerIntegration("Integration")
  , m_timerRelocalisation("Relocalisation")
  , m_timerUpdate("Update")
{
  m_denseMapper.reset(new DenseMapper(m_settings.get()));

  m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType));

  m_tracker.reset(ITMTrackerFactory::Instance().Make(m_settings->deviceType,
                                                     trackerConfig.c_str(),
                                                     imgSize_rgb,
                                                     imgsize_d,
                                                     m_lowLevelEngine.get(),
                                                     NULL,
                                                     m_scene->sceneParams));

  m_trackingController.reset(new ITMTrackingController(m_tracker.get(), m_settings.get()));

  m_trackingState.reset(new ITMTrackingState(imgsize_d, m_settings->GetMemoryType()));

  m_visualisationEngine.reset(
      ITMVisualisationEngineFactory::MakeVisualisationEngine<VoxelType, IndexType>(m_settings->deviceType));

  m_view.reset(new ITMView(calibration, imgSize_rgb, imgsize_d, m_settings->deviceType == ITMLibSettings::DEVICE_CUDA));
  m_view->depth->Clear();

  // If we initialise a m_voxelRenderState with a new variable, there is a crash after a while, probably due to never
  // using it to integrate frames in the scene.
  // Reasons are unclear.
  // Two workarounds:
  // 1. pass a renderstate from outside, and use that renderstate in spaintgui so it gets used and the variables inside
  //    are continuously set to values that prevent the crash.
  // 2. Recreate a new renderstate for each relocalisation frame. More costly (but not that much) but at least it's
  //    cleaner.
  // I chose n. 2

  //  m_voxelRenderState = voxelRenderState;
  //  m_voxelRenderState.reset(ITMRenderStateFactory<IndexType>::CreateRenderState(
  //      m_trackingController->GetTrackedImageSize(imgSize_rgb, imgsize_d),
  //      m_scene->sceneParams,
  //      m_itmLibSettings->GetMemoryType()));

  // FIXME: This global settings variable will be merged with m_settings later.
  const static std::string settingsNamespace = "ICPRefiningRelocaliser.";
  const SettingsContainer &globalSettings = SettingsContainer::instance();

  // Setup evaluation variables.
  m_saveRelocalisationPoses =
      globalSettings.get_first_value<bool>(settingsNamespace + "m_saveRelocalisationPoses", false);

  if (m_saveRelocalisationPoses)
  {
    // No "namespace" for the experiment tag.
    const std::string posesFolder =
        globalSettings.get_first_value<std::string>("experimentTag", TimeUtil::get_iso_timestamp());

    m_relocalisationPosesPathGenerator.reset(
        SequentialPathGenerator(find_subdir_from_executable("reloc_poses") / posesFolder));

    std::cout << "Saving relocalization poses in: " << m_relocalisationPosesPathGenerator->get_base_dir() << '\n';

    // Create required folders.
    fs::create_directories(m_relocalisationPosesPathGenerator->get_base_dir());
  }

  // Decide whether or not to enable the timers.
  m_timersEnabled = globalSettings.get_first_value<bool>(settingsNamespace + "m_timersEnabled", false);
}

template <typename VoxelType, typename IndexType>
ICPRefiningRelocaliser<VoxelType, IndexType>::~ICPRefiningRelocaliser()
{
  if (m_timersEnabled)
  {
    std::cout << "Integration calls: " << m_timerIntegration.count()
              << ", average duration: " << m_timerIntegration.average_duration() << '\n';

    std::cout << "Relocalisation calls: " << m_timerRelocalisation.count()
              << ", average duration: " << m_timerRelocalisation.average_duration() << '\n';

    std::cout << "Update calls: " << m_timerUpdate.count() << ", average duration: " << m_timerUpdate.average_duration()
              << '\n';
  }
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
boost::optional<Relocaliser::Result>
ICPRefiningRelocaliser<VoxelType, IndexType>::relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics) const
{
  boost::optional<ORUtils::SE3Pose> initialPose;
  return relocalise(colourImage, depthImage, depthIntrinsics, initialPose);
}

template <typename VoxelType, typename IndexType>
boost::optional<Relocaliser::Result>
ICPRefiningRelocaliser<VoxelType, IndexType>::relocalise(const ITMUChar4Image *colourImage,
                                                         const ITMFloatImage *depthImage,
                                                         const Vector4f &depthIntrinsics,
                                                         boost::optional<ORUtils::SE3Pose> &initialPose) const
{
  start_timer(m_timerRelocalisation);

  // Reset the initial pose.
  initialPose.reset();

  // Run the wrapped relocaliser.
  boost::optional<Result> relocalisationResult =
      m_innerRelocaliser->relocalise(colourImage, depthImage, depthIntrinsics);

  // If the first step of relocalisation failed, then early out.
  if (!relocalisationResult)
  {
    // Save dummy poses
    Matrix4f invalid_pose;
    invalid_pose.setValues(std::numeric_limits<float>::quiet_NaN());
    save_poses(invalid_pose, invalid_pose);
    stop_timer(m_timerRelocalisation);

    return boost::none;
  }

  // Since the inner relocaliser succeeded, copy its result into the initial pose.
  initialPose = relocalisationResult->pose;

  // Set up the view (copy directions depend on the device type).
  m_view->depth->SetFrom(depthImage,
                         m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ITMFloatImage::CUDA_TO_CUDA
                                                                               : ITMFloatImage::CPU_TO_CPU);
  m_view->rgb->SetFrom(colourImage,
                       m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ITMUChar4Image::CUDA_TO_CUDA
                                                                             : ITMUChar4Image::CPU_TO_CPU);

  // Set up the tracking state using the initial pose.
  m_trackingState->pose_d->SetFrom(initialPose.get_ptr());

  // Create a fresh renderState, to prevent a random crash after a while.
  m_voxelRenderState.reset(ITMRenderStateFactory<IndexType>::CreateRenderState(
      m_trackingController->GetTrackedImageSize(colourImage->noDims, depthImage->noDims),
      m_scene->sceneParams,
      m_settings->GetMemoryType()));

  // We need to update the list of visible blocks.
  const bool resetVisibleList = true;
  m_denseMapper->UpdateVisibleList(
      m_view.get(), m_trackingState.get(), m_scene.get(), m_voxelRenderState.get(), resetVisibleList);

  // Then perform the raycast.
  m_trackingController->Prepare(
      m_trackingState.get(), m_scene.get(), m_view.get(), m_visualisationEngine.get(), m_voxelRenderState.get());

  // Finally, run the tracker.
  m_trackingController->Track(m_trackingState.get(), m_view.get());

  // Now setup the result (if the tracking failed we are gonna return an empty optional later).
  Result refinementResult;
  refinementResult.pose.SetFrom(m_trackingState->pose_d);

  // Now, if we are in evaluation mode (we are saving the poses) and the refinement gave GOOD results, force the
  // results to POOR anyway. This is because we don't want to perform fusion whilst evaluating the testing sequence.
  refinementResult.quality =
      (!m_saveRelocalisationPoses && m_trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD)
          ? RELOCALISATION_GOOD : RELOCALISATION_POOR;

  // Save the poses.
  save_poses(initialPose->GetInvM(), refinementResult.pose.GetInvM());

  stop_timer(m_timerRelocalisation);

  // Return the result if the tracking didn't fail.
  // If it failed return an empty optional, since the initial pose was obviously bad.
  return boost::make_optional(m_trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED, refinementResult);
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::reset()
{
  m_innerRelocaliser->reset();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                                         const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  start_timer(m_timerIntegration);
  m_innerRelocaliser->train(colourImage, depthImage, depthIntrinsics, cameraPose);
  stop_timer(m_timerIntegration);
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::update()
{
  start_timer(m_timerUpdate);
  m_innerRelocaliser->update();
  stop_timer(m_timerUpdate);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::save_poses(const Matrix4f &relocalisedPose,
                                                              const Matrix4f &refinedPose) const
{
  // Early out if we don't have to save the poses.
  if (!m_saveRelocalisationPoses) return;

  // Save poses
  PosePersister::save_pose_on_thread(relocalisedPose,
                                     m_relocalisationPosesPathGenerator->make_path("pose-%06i.reloc.txt"));
  PosePersister::save_pose_on_thread(refinedPose, m_relocalisationPosesPathGenerator->make_path("pose-%06i.icp.txt"));

  // Increment counter.
  m_relocalisationPosesPathGenerator->increment_index();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::start_timer(AverageTimer &timer) const
{
  if (!m_timersEnabled) return;

#ifdef WITH_CUDA
  ORcudaSafeCall(cudaDeviceSynchronize());
#endif

  timer.start();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::stop_timer(AverageTimer &timer) const
{
  if (!m_timersEnabled) return;

#ifdef WITH_CUDA
  ORcudaSafeCall(cudaDeviceSynchronize());
#endif

  timer.stop();
}

} // namespace itmx
