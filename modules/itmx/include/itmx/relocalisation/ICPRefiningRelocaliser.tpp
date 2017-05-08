/**
 * itmx: ICPRefiningRelocaliser.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ICPRefiningRelocaliser.h"

#include <iostream>

#include <ITMLib/Core/ITMTrackingController.h>
#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Trackers/ITMTrackerFactory.h>

using namespace ITMLib;

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
  : m_relocaliser(relocaliser), m_scene(scene), m_settings(settings)
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
  m_relocaliser->integrate_rgbd_pose_pair(colourImage, depthImage, depthIntrinsics, cameraPose);
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
  // Initialise the tracking result.
  refinementDetails.refinementResult = ITMTrackingState::TRACKING_FAILED;

  // Run the wrapped relocaliser.
  refinementDetails.initialPose = m_relocaliser->relocalise(colourImage, depthImage, depthIntrinsics);

  // If the first step of relocalisation failed, then early out.
  if (!refinementDetails.initialPose)
  {
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
  m_relocaliser->update();
}

} // namespace itmx
