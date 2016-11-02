/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include <algorithm>
#include <tuple>
#include <random>

#include <boost/timer/timer.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <omp.h>

#include <DatasetRGBDInfiniTAM.hpp>

#include <libalglib/optimization.h>

#include "ITMLib/Trackers/ITMTrackerFactory.h"

#include "ocv/OpenCVUtil.h"
#include "randomforest/cuda/GPUForest_CUDA.h"
#include "randomforest/cuda/GPURansac_CUDA.h"
#include "util/PosePersister.h"

#include "Helpers.hpp"
#include "tvgutil/filesystem/PathFinder.h"
#include "tvgutil/timing/TimeUtil.h"

using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace RelocLib;
using namespace tvgutil;

//#define ENABLE_TIMERS
//#define VISUALIZE_INLIERS
#define SAVE_RELOC_POSES
#define USE_FERN_RELOCALISER

namespace spaint
{

//#################### CONSTRUCTORS ####################

SLAMComponentWithScoreForest::SLAMComponentWithScoreForest(
    const SLAMContext_Ptr& context, const std::string& sceneID,
    const ImageSourceEngine_Ptr& imageSourceEngine, TrackerType trackerType,
    const std::vector<std::string>& trackerParams, MappingMode mappingMode,
    TrackingMode trackingMode) :
    SLAMComponent(context, sceneID, imageSourceEngine, trackerType,
        trackerParams, mappingMode, trackingMode)
{
  m_dataset.reset(
      new DatasetRGBDInfiniTAM(
//          "/home/tcavallari/code/scoreforests/apps/TrainAndTest/SettingsDatasetRGBDInfiniTAMDesk.yml",
//          "/home/tcavallari/code/scoreforests/apps/TrainAndTest/SettingsDatasetRGBD7ScenesChessOnline.yml",
          "/home/tcavallari/code/scoreforests/apps/TrainAndTest/SettingsDatasetRGBD7ScenesOfficeOnline.yml",
          "/media/data/", 5, 1.0, "DFBP", true, 0, false, 42));

  m_dataset->LoadForest();
//  m_dataset->ResetNodeAndLeaves();

  m_featureExtractor =
      FeatureCalculatorFactory::make_rgbd_patch_feature_calculator(
          ITMLib::ITMLibSettings::DEVICE_CUDA);
  m_featureImage.reset(new RGBDPatchFeatureImage(Vector2i(0, 0), true, true)); // Dummy size just to allocate the container
  m_predictionsImage.reset(
      new GPUForestPredictionsImage(Vector2i(0, 0), true, true)); // Dummy size just to allocate the container

  m_gpuForest.reset(new GPUForest_CUDA(*m_dataset->GetForest()));
  m_gpuForest->reset_predictions();

  m_gpuRansac.reset(new GPURansac_CUDA());

  // Refinement ICP tracker
  const Settings_CPtr& settings = m_context->get_settings();
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const Vector2i& depthImageSize = slamState->get_depth_image_size();
  const Vector2i& rgbImageSize = slamState->get_rgb_image_size();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

#ifdef SAVE_RELOC_POSES
  const std::string refineParams =
      "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

  m_refineTracker.reset(
      ITMTrackerFactory<SpaintVoxel, ITMVoxelIndex>::Instance().Make(
          refineParams.c_str(), rgbImageSize, depthImageSize, settings.get(),
          m_lowLevelEngine.get(), NULL, voxelScene.get()));

  const std::string poses_folder =
      m_context->get_tag().empty() ?
          TimeUtil::get_iso_timestamp() : m_context->get_tag();

  m_sequentialPathGenerator.reset(
      SequentialPathGenerator(
          find_subdir_from_executable("reloc_poses") / poses_folder));

  std::cout << "Saving relocalization poses in: "
      << m_sequentialPathGenerator->get_base_dir() << std::endl;
  boost::filesystem::create_directories(
      m_sequentialPathGenerator->get_base_dir());
#endif
}

//#################### DESTRUCTOR ####################
SLAMComponentWithScoreForest::~SLAMComponentWithScoreForest()
{
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

#ifndef USE_FERN_RELOCALISER

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(
    TrackingResult trackingResult)
{
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const ITMFloatImage_Ptr inputDepthImage(
      new ITMFloatImage(slamState->get_depth_image_size(), true, true));
  inputDepthImage->SetFrom(slamState->get_view()->depth,
      ORUtils::MemoryBlock<float>::CUDA_TO_CUDA);

  const ITMUChar4Image_Ptr inputRGBImage(
      new ITMUChar4Image(slamState->get_rgb_image_size(), true, true));
  inputRGBImage->SetFrom(slamState->get_view()->rgb,
      ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA);

  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();

  const View_Ptr& view = slamState->get_view();

  const Vector4f depthIntrinsics =
  view->calib.intrinsics_d.projectionParamsSimple.all;

  if (trackingResult == TrackingResult::TRACKING_FAILED)
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "relocalization, overall: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    if (m_lowLevelEngine->CountValidDepths(inputDepthImage.get())
        < m_gpuRansac->get_min_nb_required_points())
    {
      std::cout
      << "Number of valid depth pixels insufficient to perform relocalization."
      << std::endl;

      if (m_sequentialPathGenerator)
      {
        m_sequentialPathGenerator->increment_index();
      }
      return trackingResult;
    }

    compute_features(inputRGBImage, inputDepthImage, depthIntrinsics);
    evaluate_forest();
    boost::optional<PoseCandidate> pose_candidate = m_gpuRansac->estimate_pose(
        m_featureImage, m_predictionsImage);

    if (pose_candidate)
    {
//      std::cout << "The final pose is:" << pose_candidate->cameraPose
//          << "\n and has " << pose_candidate->inliers.size() << " inliers."
//          << std::endl;

#ifdef VISUALIZE_INLIERS
      cv::Mat inliers = cv::Mat::zeros(
          cv::Size(m_featureImage->noDims.width, m_featureImage->noDims.height),
          CV_32FC1);
      inliers.setTo(std::numeric_limits<float>::quiet_NaN());

      for (size_t i = 0; i < pose_candidate->inliers.size(); ++i)
      {
        int idx = pose_candidate->inliers[i].linearIdx;
        float energy = pose_candidate->inliers[i].energy;

        int x = idx % m_featureImage->noDims.width;
        int y = idx / m_featureImage->noDims.width;

        inliers.at<float>(cv::Point(x, y)) = energy;
      }

      double min, max;
      cv::minMaxIdx(inliers, &min, &max);
      std::cout << "Min energy: " << min << " - MAx energy: " << max
      << std::endl;

      cv::normalize(inliers, inliers, 0.0, 1.0, cv::NORM_MINMAX);
      inliers = 1.f - inliers;

      cv::imshow("Inliers mask", inliers);
      cv::waitKey(1);
#endif

      trackingState->pose_d->SetInvM(pose_candidate->cameraPose);

      const VoxelRenderState_Ptr& liveVoxelRenderState =
      slamState->get_live_voxel_render_state();
      const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();
      const bool resetVisibleList = true;
      m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(),
          voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
      prepare_for_tracking(TRACK_VOXELS);
#ifdef SAVE_RELOC_POSES
      m_refineTracker->TrackCamera(trackingState.get(), view.get());
#else
      m_trackingController->Track(trackingState.get(), view.get());
#endif
      trackingResult = trackingState->trackerResult;

//      std::cout << "Refinement: "
//          << (trackingState->trackerResult == TrackingResult::TRACKING_GOOD ?
//              "SUCCESS" : "FAIL") << "\n Refined pose:\n"
//          << trackingState->pose_d->GetInvM() << std::endl;

      if (m_sequentialPathGenerator)
      {
        // Save poses
        PosePersister::save_pose_on_thread(pose_candidate->cameraPose,
            m_sequentialPathGenerator->make_path("pose-%06i.reloc.txt"));
        PosePersister::save_pose_on_thread(trackingState->pose_d->GetInvM(),
            m_sequentialPathGenerator->make_path("pose-%06i.icp.txt"));

        const Matrix4f final_pose =
        trackingResult == TrackingResult::TRACKING_GOOD ?
        trackingState->pose_d->GetInvM() : pose_candidate->cameraPose;

        PosePersister::save_pose_on_thread(final_pose,
            m_sequentialPathGenerator->make_path("pose-%06i.final.txt"));
      }

#ifdef SAVE_RELOC_POSES
      trackingResult = TrackingResult::TRACKING_POOR;
#endif
    }
    else
    {
      std::cout << "Cannot estimate a pose candidate." << std::endl;

      if (m_sequentialPathGenerator)
      {
        // Save dummy poses
        Matrix4f invalid_pose;
        invalid_pose.setValues(std::numeric_limits<float>::quiet_NaN());

        PosePersister::save_pose_on_thread(invalid_pose,
            m_sequentialPathGenerator->make_path("pose-%06i.reloc.txt"));
        PosePersister::save_pose_on_thread(invalid_pose,
            m_sequentialPathGenerator->make_path("pose-%06i.icp.txt"));
        PosePersister::save_pose_on_thread(invalid_pose,
            m_sequentialPathGenerator->make_path("pose-%06i.final.txt"));
      }
    }

    if (m_sequentialPathGenerator)
    {
      m_sequentialPathGenerator->increment_index();
    }
  }
  else if (trackingResult == TrackingResult::TRACKING_GOOD)
  {
    Matrix4f invCameraPose = trackingState->pose_d->GetInvM();
    compute_features(inputRGBImage, inputDepthImage, depthIntrinsics,
        invCameraPose);

#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "add features to forest: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    m_gpuForest->add_features_to_forest(m_featureImage);
  }

  return trackingResult;
}

#else

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(
    TrackingResult trackingResult)
{
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const VoxelRenderState_Ptr& liveVoxelRenderState =
      slamState->get_live_voxel_render_state();
  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();
  const View_Ptr& view = slamState->get_view();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

  // Copy the current depth input across to the CPU for use by the relocaliser.
  view->depth->UpdateHostFromDevice();

  // Decide whether or not the relocaliser should consider using this frame as a keyframe.
  bool considerKeyframe = false;
  if (trackingResult == ITMTrackingState::TRACKING_GOOD)
  {
    if (m_keyframeDelay == 0)
      considerKeyframe = true;
    else
      --m_keyframeDelay;
  }

  // Process the current depth image using the relocaliser. This attempts to find the nearest keyframe (if any)
  // that is currently in the database, and may add the current frame as a new keyframe if the tracking has been
  // good for some time and the current frame differs sufficiently from the existing keyframes.
  int nearestNeighbour;
  int keyframeID = m_relocaliser->ProcessFrame(view->depth, 1,
      &nearestNeighbour, NULL, considerKeyframe);

  if (keyframeID >= 0)
  {
    // If the relocaliser added the current frame as a new keyframe, store its pose in the pose database.
    // Note that a new keyframe will only have been added if the tracking quality for this frame was good.
    m_poseDatabase->storePose(keyframeID, *trackingState->pose_d, 0);
  }
  else if (trackingResult == ITMTrackingState::TRACKING_FAILED
      && nearestNeighbour != -1)
  {
    // If the tracking failed but a nearest keyframe was found by the relocaliser, reset the pose to that
    // of the keyframe and rerun the tracker for this frame.
    ORUtils::SE3Pose relocPose =
        m_poseDatabase->retrievePose(nearestNeighbour).pose;

    trackingState->pose_d->SetFrom(&relocPose);

    const bool resetVisibleList = true;
    m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(),
        voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
    prepare_for_tracking(TRACK_VOXELS);
#ifdef SAVE_RELOC_POSES
    m_refineTracker->TrackCamera(trackingState.get(), view.get());
#else
    m_trackingController->Track(trackingState.get(), view.get());
#endif
    trackingResult = trackingState->trackerResult;

    if (m_sequentialPathGenerator)
    {
      // Save poses
      PosePersister::save_pose_on_thread(relocPose.GetInvM(),
          m_sequentialPathGenerator->make_path("pose-%06i.reloc.txt"));
      PosePersister::save_pose_on_thread(trackingState->pose_d->GetInvM(),
          m_sequentialPathGenerator->make_path("pose-%06i.icp.txt"));

      const Matrix4f final_pose =
          trackingResult == TrackingResult::TRACKING_GOOD ?
              trackingState->pose_d->GetInvM() : relocPose.GetInvM();

      PosePersister::save_pose_on_thread(final_pose,
          m_sequentialPathGenerator->make_path("pose-%06i.final.txt"));

      m_sequentialPathGenerator->increment_index();
    }

#ifdef SAVE_RELOC_POSES
    trackingResult = TrackingResult::TRACKING_POOR;
#endif

    // Set the number of frames for which the tracking quality must be good before the relocaliser can consider
    // adding a new keyframe.
    m_keyframeDelay = 10;
  }

  return trackingResult;
}

#endif

//#################### PROTECTED MEMBER FUNCTIONS ####################

void SLAMComponentWithScoreForest::compute_features(
    const ITMUChar4Image_CPtr &inputRgbImage,
    const ITMFloatImage_CPtr &inputDepthImage, const Vector4f &depthIntrinsics)
{
  Matrix4f identity;
  identity.setIdentity();

  compute_features(inputRgbImage, inputDepthImage, depthIntrinsics, identity);
}

void SLAMComponentWithScoreForest::compute_features(
    const ITMUChar4Image_CPtr &inputRgbImage,
    const ITMFloatImage_CPtr &inputDepthImage, const Vector4f &depthIntrinsics,
    const Matrix4f &invCameraPose)
{
#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6,
      "computing features on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
  m_featureExtractor->ComputeFeature(inputRgbImage, inputDepthImage,
      depthIntrinsics, m_featureImage, invCameraPose);

}

void SLAMComponentWithScoreForest::evaluate_forest()
{
#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6,
      "evaluating forest on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
  m_gpuForest->evaluate_forest(m_featureImage, m_predictionsImage);
}

}
