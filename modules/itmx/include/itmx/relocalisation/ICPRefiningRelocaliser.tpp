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
#include <tvgutil/misc/GlobalParameters.h>
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
  : m_relocaliser(relocaliser)
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

  const static std::string parametersNamespace = "ICPRefiningRelocaliser.";
  const GlobalParameters &globalParams = GlobalParameters::instance();

  // Setup evaluation variables.
  m_saveRelocalisationPoses =
      globalParams.get_first_value<bool>(parametersNamespace + "m_saveRelocalisationPoses", false);

  if (m_saveRelocalisationPoses)
  {
    // No "namespace" for the experiment tag.
    const std::string posesFolder =
        globalParams.get_first_value<std::string>("experimentTag", TimeUtil::get_iso_timestamp());

    m_relocalisationPosesPathGenerator.reset(
        SequentialPathGenerator(find_subdir_from_executable("reloc_poses") / posesFolder));

    std::cout << "Saving relocalization poses in: " << m_relocalisationPosesPathGenerator->get_base_dir() << '\n';

    // Create required folders.
    fs::create_directories(m_relocalisationPosesPathGenerator->get_base_dir());
  }

  // Decide whether or not to enable the timers.
  m_timersEnabled = globalParams.get_first_value<bool>(parametersNamespace + "m_timersEnabled", false);
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

    std::cout << "Update calls: " << m_timerUpdate.count()
              << ", average duration: " << m_timerUpdate.average_duration() << '\n';
  }
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
Relocaliser_Ptr ICPRefiningRelocaliser<VoxelType, IndexType>::get_inner_relocaliser() const
{
  return m_relocaliser;
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::integrate_rgbd_pose_pair(const ITMUChar4Image *colourImage,
                                                                            const ITMFloatImage *depthImage,
                                                                            const Vector4f &depthIntrinsics,
                                                                            const ORUtils::SE3Pose &cameraPose)
{
  start_timer(m_timerIntegration);
  m_relocaliser->integrate_rgbd_pose_pair(colourImage, depthImage, depthIntrinsics, cameraPose);
  stop_timer(m_timerIntegration);
}

template <typename VoxelType, typename IndexType>
boost::optional<ORUtils::SE3Pose> ICPRefiningRelocaliser<VoxelType, IndexType>::relocalise(
    const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f &depthIntrinsics)
{
  RefinementDetails details;
  return relocalise(colourImage, depthImage, depthIntrinsics, details);
}

template <typename VoxelType, typename IndexType>
boost::optional<ORUtils::SE3Pose>
    ICPRefiningRelocaliser<VoxelType, IndexType>::relocalise(const ITMUChar4Image *colourImage,
                                                             const ITMFloatImage *depthImage,
                                                             const Vector4f &depthIntrinsics,
                                                             RefiningRelocaliser::RefinementDetails &refinementDetails)
{
  start_timer(m_timerRelocalisation);

  // Initialise the tracking result.
  refinementDetails.refinementResult = ITMTrackingState::TRACKING_FAILED;

  // Run the wrapped relocaliser.
  refinementDetails.initialPose = m_relocaliser->relocalise(colourImage, depthImage, depthIntrinsics);

  // If the first step of relocalisation failed, then early out.
  if (!refinementDetails.initialPose)
  {
    // Save dummy poses
    Matrix4f invalid_pose;
    invalid_pose.setValues(std::numeric_limits<float>::quiet_NaN());
    save_poses(invalid_pose, invalid_pose);
    stop_timer(m_timerRelocalisation);

    return boost::none;
  }

  // Set up the view (copy directions depend on the device type).
  m_view->depth->SetFrom(depthImage,
                         m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ITMFloatImage::CUDA_TO_CUDA
                                                                               : ITMFloatImage::CPU_TO_CPU);
  m_view->rgb->SetFrom(colourImage,
                       m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ITMUChar4Image::CUDA_TO_CUDA
                                                                             : ITMUChar4Image::CPU_TO_CPU);

  // Set up the tracking state using the relocalised pose.
  m_trackingState->pose_d->SetFrom(refinementDetails.initialPose.get_ptr());

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

  // Now copy the tracking state in the details struct.
  refinementDetails.refinementResult = m_trackingState->trackerResult;

  // Save the poses.
  save_poses(refinementDetails.initialPose->GetInvM(), m_trackingState->pose_d->GetInvM());

  // Now, if we are in evaluation mode (we are saving the poses) and the refinement gave GOOD results, force the
  // tracking result to poor. This is because we don't want to perform fusion whilst evaluating the testing sequence.
  if (m_saveRelocalisationPoses && refinementDetails.refinementResult == ITMTrackingState::TRACKING_GOOD)
  {
    refinementDetails.refinementResult = ITMTrackingState::TRACKING_POOR;
  }

  stop_timer(m_timerRelocalisation);

  // Return the refined pose if the tracking didn't fail.
  // If it failed return an empty optional, since the initial pose was obviously bad.
  return boost::make_optional(refinementDetails.refinementResult != ITMTrackingState::TRACKING_FAILED,
                              *m_trackingState->pose_d);
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::reset()
{
  m_relocaliser->reset();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::update()
{
  start_timer(m_timerUpdate);
  m_relocaliser->update();
  stop_timer(m_timerUpdate);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::save_poses(const Matrix4f &relocalisedPose,
                                                              const Matrix4f &refinedPose)
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
void ICPRefiningRelocaliser<VoxelType, IndexType>::start_timer(AverageTimer &timer)
{
  if (!m_timersEnabled) return;

#ifdef WITH_CUDA
  ORcudaSafeCall(cudaDeviceSynchronize());
#endif

  timer.start();
}

template <typename VoxelType, typename IndexType>
void ICPRefiningRelocaliser<VoxelType, IndexType>::stop_timer(AverageTimer &timer)
{
  if (!m_timersEnabled) return;

#ifdef WITH_CUDA
  ORcudaSafeCall(cudaDeviceSynchronize());
#endif

  timer.stop();
}

} // namespace itmx
