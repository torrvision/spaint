/**
 * itmx: ICPRefiningRelocaliser.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ICPRefiningRelocaliser.h"

#include <iostream>
#include <stdexcept>

#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/misc/SettingsContainer.h>
#include <tvgutil/timing/TimeUtil.h>

#include "../persistence/PosePersister.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

template <typename VoxelType, typename IndexType>
ICPRefiningRelocaliser<VoxelType,IndexType>::ICPRefiningRelocaliser(const Relocaliser_Ptr& innerRelocaliser, const Tracker_Ptr& tracker,
                                                                    const Vector2i& rgbImageSize, const Vector2i& depthImageSize, const ITMLib::ITMRGBDCalib& calib,
                                                                    const Scene_Ptr& scene, const DenseMapper_Ptr& denseVoxelMapper, const Settings_CPtr& settings,
                                                                    const LowLevelEngine_CPtr& lowLevelEngine, const VisualisationEngine_CPtr& visualisationEngine)
: RefiningRelocaliser(innerRelocaliser),
  m_denseVoxelMapper(denseVoxelMapper),
  m_lowLevelEngine(lowLevelEngine),
  m_scene(scene),
  m_settings(settings),
  m_timerRelocalisation("Relocalisation"),
  m_timerTraining("Training"),
  m_timerUpdate("Update"),
  m_tracker(tracker),
  m_visualisationEngine(visualisationEngine)
{
  // Construct the tracking controller, tracking state and view.
  m_trackingController.reset(new ITMTrackingController(m_tracker.get(), m_settings.get()));
  m_trackingState.reset(new ITMTrackingState(depthImageSize, m_settings->GetMemoryType()));
  m_view.reset(new ITMView(calib, rgbImageSize, depthImageSize, m_settings->deviceType == ITMLibSettings::DEVICE_CUDA));

  // Configure the relocaliser based on the settings that have been passed in.
  const static std::string settingsNamespace = "ICPRefiningRelocaliser.";
  m_savePoses = m_settings->get_first_value<bool>(settingsNamespace + "saveRelocalisationPoses", false);
  m_timersEnabled = m_settings->get_first_value<bool>(settingsNamespace + "timersEnabled", false);

  if(m_savePoses)
  {
    // Get the (global) experiment tag.
    const std::string experimentTag = m_settings->get_first_value<std::string>("experimentTag", tvgutil::TimeUtil::get_iso_timestamp());

    // Determine the directory to which to save the poses and make sure that it exists.
    m_posePathGenerator.reset(tvgutil::SequentialPathGenerator(tvgutil::find_subdir_from_executable("reloc_poses") / experimentTag));
    boost::filesystem::create_directories(m_posePathGenerator->get_base_dir());

    // Output the directory we're using (for debugging purposes).
    std::cout << "Saving relocalisation poses in: " << m_posePathGenerator->get_base_dir() << '\n';
  }
}

//#################### DESTRUCTOR ####################

template <typename VoxelType, typename IndexType>
ICPRefiningRelocaliser<VoxelType,IndexType>::~ICPRefiningRelocaliser()
{
  if(m_timersEnabled)
  {
    std::cout << "Training calls: " << m_timerTraining.count() << ", average duration: " << m_timerTraining.average_duration() << '\n';
    std::cout << "Relocalisation calls: " << m_timerRelocalisation.count() << ", average duration: " << m_timerRelocalisation.average_duration() << '\n';
    std::cout << "Update calls: " << m_timerUpdate.count() << ", average duration: " << m_timerUpdate.average_duration() << '\n';
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
boost::optional<Relocaliser::Result>
ICPRefiningRelocaliser<VoxelType,IndexType>::relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                                        const Vector4f& depthIntrinsics) const
{
  boost::optional<ORUtils::SE3Pose> initialPose;
  return relocalise(colourImage, depthImage, depthIntrinsics, initialPose);
}

template <typename VoxelType, typename IndexType>
boost::optional<Relocaliser::Result>
ICPRefiningRelocaliser<VoxelType,IndexType>::relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                                        const Vector4f& depthIntrinsics, boost::optional<ORUtils::SE3Pose>& initialPose) const
{
  start_timer(m_timerRelocalisation);

  // Reset the initial pose.
  initialPose.reset();

  // Run the inner relocaliser. If it fails, save dummy poses and early out.
  boost::optional<Result> relocalisationResult = m_innerRelocaliser->relocalise(colourImage, depthImage, depthIntrinsics);
  if(!relocalisationResult)
  {
    Matrix4f invalidPose;
    invalidPose.setValues(std::numeric_limits<float>::quiet_NaN());
    save_poses(invalidPose, invalidPose);
    stop_timer(m_timerRelocalisation);
    return boost::none;
  }

  // Since the inner relocaliser succeeded, copy its result into the initial pose.
  initialPose = relocalisationResult->pose;

  // Copy the depth and RGB images into the view.
  m_view->depth->SetFrom(depthImage, m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ITMFloatImage::CUDA_TO_CUDA : ITMFloatImage::CPU_TO_CPU);
  m_view->rgb->SetFrom(colourImage, m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ITMUChar4Image::CUDA_TO_CUDA : ITMUChar4Image::CPU_TO_CPU);

  // Create a fresh render state ready for raycasting.
  // FIXME: It would be nicer to simply create the render state once and then reuse it, but unfortunately this leads
  //        to the program randomly crashing after a while. The crash may be occurring because we don't use this render
  //        state to integrate frames into the scene, but we haven't been able to pin this down yet. As a result, we
  //        currently create a fresh render state each time as a workaround. A mildly less costly alternative might
  //        be to pass in a render state that is being used elsewhere and reuse it here, but that feels messier.
  m_voxelRenderState.reset(ITMRenderStateFactory<IndexType>::CreateRenderState(
    m_trackingController->GetTrackedImageSize(colourImage->noDims, depthImage->noDims),
    m_scene->sceneParams,
    m_settings->GetMemoryType()
  ));

  // Set up the tracking state using the initial pose.
  m_trackingState->pose_d->SetFrom(initialPose.get_ptr());

  // Update the list of visible blocks.
  const bool resetVisibleList = true;
  m_denseVoxelMapper->UpdateVisibleList(m_view.get(), m_trackingState.get(), m_scene.get(), m_voxelRenderState.get(), resetVisibleList);

  // Raycast from the initial pose to prepare for tracking.
  m_trackingController->Prepare(m_trackingState.get(), m_scene.get(), m_view.get(), m_visualisationEngine.get(), m_voxelRenderState.get());

  // Run the tracker to refine the initial pose.
  m_trackingController->Track(m_trackingState.get(), m_view.get());

  // Save the poses.
  save_poses(initialPose->GetInvM(), m_trackingState->pose_d->GetInvM());

  // Set up the result.
  boost::optional<Result> refinementResult;
  if(m_trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
  {
    refinementResult.reset(Result());
    refinementResult->pose.SetFrom(m_trackingState->pose_d);
    refinementResult->quality = m_trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD ? RELOCALISATION_GOOD : RELOCALISATION_POOR;

    // If we are in evaluation mode (we are saving the poses), force the quality to POOR to prevent fusion whilst evaluating the testing sequence.
    if(m_savePoses) refinementResult->quality = RELOCALISATION_POOR;
  }

  stop_timer(m_timerRelocalisation);

  return refinementResult;
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::reset()
{
  m_innerRelocaliser->reset();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                                        const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  start_timer(m_timerTraining);
  m_innerRelocaliser->train(colourImage, depthImage, depthIntrinsics, cameraPose);
  stop_timer(m_timerTraining);
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::update()
{
  start_timer(m_timerUpdate);
  m_innerRelocaliser->update();
  stop_timer(m_timerUpdate);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::save_poses(const Matrix4f& relocalisedPose, const Matrix4f& refinedPose) const
{
  if(!m_savePoses) return;

  PosePersister::save_pose_on_thread(relocalisedPose, m_posePathGenerator->make_path("pose-%06i.reloc.txt"));
  PosePersister::save_pose_on_thread(refinedPose, m_posePathGenerator->make_path("pose-%06i.icp.txt"));
  m_posePathGenerator->increment_index();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::start_timer(AverageTimer& timer) const
{
  if(!m_timersEnabled) return;

#ifdef WITH_CUDA
  ORcudaSafeCall(cudaDeviceSynchronize());
#endif

  timer.start();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType,IndexType>::stop_timer(AverageTimer& timer) const
{
  if(!m_timersEnabled) return;

#ifdef WITH_CUDA
  ORcudaSafeCall(cudaDeviceSynchronize());
#endif

  timer.stop();
}

}
