/**
 * spaint: SLAMComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include "ITMLib/Trackers/ITMTrackerFactory.h"

#include "randomforest/cuda/GPUForest_CUDA.h"
#include "randomforest/cuda/PreemptiveRansac_CUDA.h"
#include "util/MemoryBlockFactory.h"
#include "util/PosePersister.h"

#include "tvgutil/filesystem/PathFinder.h"

//#define ENABLE_TIMERS
//#define VISUALIZE_INLIERS
#define SAVE_RELOC_POSES
//#define SAVE_LEAF_MODES
//#define SAVE_INLIERS
//#define USE_FERN_RELOCALISER
#define RELOCALISE_EVERY_TRAINING_FRAME

#ifdef ENABLE_TIMERS
#include <boost/timer/timer.hpp>
#endif

#ifdef VISUALIZE_INLIERS
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif

#ifdef SAVE_RELOC_POSES
#include "tvgutil/timing/TimeUtil.h"
#endif

namespace bf = boost::filesystem;
using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace RelocLib;
using namespace tvgutil;

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
  const Settings_CPtr& settings = m_context->get_settings();
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_featureExtractor =
      FeatureCalculatorFactory::make_rgbd_patch_feature_calculator(
          settings->deviceType);
  m_featureImage = mbf.make_image<RGBDPatchFeature>(Vector2i(0, 0)); // Dummy size just to allocate the container
  m_predictionsImage = mbf.make_image<GPUForestPrediction>(Vector2i(0, 0)); // Dummy size just to allocate the container

  const bf::path relocalizationForestPath = bf::path(
      m_context->get_resources_dir()) / "DefaultRelocalizationForest.rf";

  std::cout << "Loading relocalization forest from: "
      << relocalizationForestPath << '\n';
  m_gpuForest.reset(new GPUForest_CUDA(relocalizationForestPath.string()));
  m_updateForestModesEveryFrame = true;

  m_preemptiveRansac.reset(new PreemptiveRansac_CUDA());
//  m_preemptiveRansac.reset(new PreemptiveRansac());

  // Refinement ICP tracker
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const Vector2i& depthImageSize = slamState->get_depth_image_size();
  const Vector2i& rgbImageSize = slamState->get_rgb_image_size();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

  const std::string refineParams =
      "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

  m_refineTracker.reset(
      ITMTrackerFactory<SpaintVoxel, ITMVoxelIndex>::Instance().Make(
          refineParams.c_str(), rgbImageSize, depthImageSize, settings.get(),
          m_lowLevelEngine.get(), NULL, voxelScene.get()));

  m_timeRelocalizer = true;
  m_learningCalls = 0;
  m_learningTimes.clear();
  m_relocalizationCalls = 0;
  m_relocalizationTimes.clear();

#ifdef SAVE_RELOC_POSES
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
  if (m_timeRelocalizer)
  {
    std::cout << "Relocalizer called " << m_relocalizationCalls << "times.\n";

    if (m_relocalizationCalls > 0)
    {
      std::cout << "\tTotal time: "
          << boost::timer::format(m_relocalizationTimes, 6);
      // Average times
      m_relocalizationTimes.system /= m_relocalizationCalls;
      m_relocalizationTimes.user /= m_relocalizationCalls;
      m_relocalizationTimes.wall /= m_relocalizationCalls;
      std::cout << "\tAverage time: "
          << boost::timer::format(m_relocalizationTimes, 6);
    }

    std::cout << "Learner called " << m_learningCalls << "times.\n";
    if (m_learningCalls > 0)
    {
      std::cout << "\tTotal time: " << boost::timer::format(m_learningTimes, 6);
      // Average times
      m_learningTimes.system /= m_learningCalls;
      m_learningTimes.user /= m_learningCalls;
      m_learningTimes.wall /= m_learningCalls;
      std::cout << "\tAverage time: "
          << boost::timer::format(m_learningTimes, 6);
    }
  }
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

#ifndef USE_FERN_RELOCALISER

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(
    TrackingResult trackingResult)
{
  boost::optional<boost::timer::cpu_timer> relocalizationTimer;

  if (m_timeRelocalizer)
  {
    ORcudaSafeCall(cudaDeviceSynchronize());
    relocalizationTimer = boost::timer::cpu_timer();
  }

#ifdef RELOCALISE_EVERY_TRAINING_FRAME
  const bool performRelocalization = true;
  const bool performLearning = true;
#else
  const bool performRelocalization = trackingResult
  == TrackingResult::TRACKING_FAILED;
  const bool performLearning = trackingResult == TrackingResult::TRACKING_GOOD;
#endif

  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const ITMFloatImage *inputDepthImage = slamState->get_view()->depth;
  const ITMUChar4Image *inputRGBImage = slamState->get_view()->rgb;

  const TrackingState_Ptr& trackingState = slamState->get_tracking_state();
  const SE3Pose trackedPose(*trackingState->pose_d);

  const View_Ptr& view = slamState->get_view();

  const Vector4f depthIntrinsics =
      view->calib.intrinsics_d.projectionParamsSimple.all;

  if (m_updateForestModesEveryFrame && !performLearning)
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "update forest, overall: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_gpuForest->update_forest();
//    cudaDeviceSynchronize();
  }

  if (performRelocalization)
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "relocalization, overall: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

#ifdef SAVE_LEAF_MODES

    // Leaf indices selected randomly during the forest conversion step
    std::vector<size_t> predictionIndices
    { 5198, 447, 5438, 7355, 1649};

//    std::vector<size_t> predictionIndices
//    { 5198, 447, 5438, 1664, 4753 };

    // For each prediction print centroids, covariances, nbInliers
    for (size_t treeIdx = 0; treeIdx < predictionIndices.size(); ++treeIdx)
    {
      const GPUForestPrediction p = m_gpuForest->get_prediction(treeIdx,
          predictionIndices[treeIdx]);
      std::cout << p.nbModes << ' ' << predictionIndices[treeIdx] << '\n';
      for (int modeIdx = 0; modeIdx < p.nbModes; ++modeIdx)
      {
        const GPUForestMode &m = p.modes[modeIdx];
        std::cout << m.nbInliers << ' ' << m.position.x << ' ' << m.position.y
        << ' ' << m.position.z << ' ';

        // Invet and transpose the covariance to print it in row-major
        Matrix3f posCovariance;
        m.positionInvCovariance.inv(posCovariance);
        posCovariance = posCovariance.t();

        for (int i = 0; i < 9; ++i)
        std::cout << posCovariance.m[i] << ' ';
        std::cout << '\n';
      }
      std::cout << '\n';
    }

    // Done for this test
    exit(0);
#endif

    if (m_lowLevelEngine->CountValidDepths(inputDepthImage)
        < m_preemptiveRansac->get_min_nb_required_points())
    {
//      std::cout
//          << "Number of valid depth pixels insufficient to perform relocalization."
//          << std::endl;

      if (m_sequentialPathGenerator)
      {
        m_sequentialPathGenerator->increment_index();
      }
      return trackingResult;
    }

    compute_features(inputRGBImage, inputDepthImage, depthIntrinsics);
    evaluate_forest();
    boost::optional<PoseCandidate> pose_candidate =
        m_preemptiveRansac->estimate_pose(m_featureImage, m_predictionsImage);

    if (pose_candidate)
    {
//      std::cout << "The final pose is:" << pose_candidate->cameraPose
//          << "\n and has " << pose_candidate->nbInliers << " inliers."
//          << std::endl;

#ifdef VISUALIZE_INLIERS
      cv::Mat inliers = cv::Mat::zeros(
          cv::Size(m_featureImage->noDims.width, m_featureImage->noDims.height),
          CV_32FC1);
      inliers.setTo(std::numeric_limits<float>::quiet_NaN());

      for (int i = 0; i < pose_candidate->nbInliers; ++i)
      {
        int idx = pose_candidate->inliers[i].linearIdx;
        float energy = pose_candidate->inliers[i].energy;

        int x = idx % m_featureImage->noDims.width;
        int y = idx / m_featureImage->noDims.width;

        inliers.at<float>(cv::Point(x, y)) = energy;
      }

//      double min, max;
//      cv::minMaxIdx(inliers, &min, &max);
//      std::cout << "Min energy: " << min << " - MAx energy: " << max
//          << std::endl;

      cv::Mat inliersImg;
      cv::normalize(inliers, inliersImg, 0.0, 1.0, cv::NORM_MINMAX);
      inliersImg = 1.f - inliersImg;

      // Convert the image in cv_8uc1 (NaNs become 0)
//      inliers.convertTo(inliers, CV_8U, 255.0);
//      cv::applyColorMap(inliers, inliers, cv::COLORMAP_JET);
      cv::resize(inliersImg, inliersImg,
          cv::Size(inputRGBImage->noDims.width, inputRGBImage->noDims.height),
          -1, -1, cv::INTER_NEAREST);

      cv::imshow("Inliers mask", inliersImg);
      cv::waitKey(1);

#ifdef SAVE_INLIERS
      if (m_sequentialPathGenerator)
      {
        std::string inliersFilename = m_sequentialPathGenerator->make_path(
            "ransac-%06i.inliers.dat").string();

        std::ofstream out(inliersFilename);
        for (int j = 0; j < inliers.rows; ++j)
        {
          float *rowPtr = inliers.ptr<float>(j);
          for (int i = 0; i < inliers.cols; ++i)
          {
            out << rowPtr[i] << ' ';
          }
          out << '\n';
        }
//        cv::imwrite(inliersFilename, inliers);
      }
#endif
#endif

      trackingState->pose_d->SetInvM(pose_candidate->cameraPose);

      const VoxelRenderState_Ptr& liveVoxelRenderState =
          slamState->get_live_voxel_render_state();
      const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();
      const bool resetVisibleList = true;
      m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(),
          voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
      prepare_for_tracking(TRACK_VOXELS);
      m_refineTracker->TrackCamera(trackingState.get(), view.get());
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

    if (relocalizationTimer)
    {
      ORcudaSafeCall(cudaDeviceSynchronize());
      relocalizationTimer->stop();

      boost::timer::cpu_times elapsedTimes = relocalizationTimer->elapsed();
      m_relocalizationTimes.system += elapsedTimes.system;
      m_relocalizationTimes.wall += elapsedTimes.wall;
      m_relocalizationTimes.user += elapsedTimes.user;
      ++m_relocalizationCalls;
    }
  }

  if (performLearning)
  {
    Matrix4f invCameraPose = trackedPose.GetInvM();
    compute_features(inputRGBImage, inputDepthImage, depthIntrinsics,
        invCameraPose);

#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "add features to forest: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    m_gpuForest->add_features_to_forest(m_featureImage);

    if (relocalizationTimer)
    {
      ORcudaSafeCall(cudaDeviceSynchronize());
      relocalizationTimer->stop();

      boost::timer::cpu_times elapsedTimes = relocalizationTimer->elapsed();
      m_learningTimes.system += elapsedTimes.system;
      m_learningTimes.wall += elapsedTimes.wall;
      m_learningTimes.user += elapsedTimes.user;
      ++m_learningCalls;
    }
  }

#ifdef RELOCALISE_EVERY_TRAINING_FRAME
  // Restore tracked pose
  trackingState->pose_d->SetFrom(&trackedPose);
  trackingResult = TrackingResult::TRACKING_GOOD;
#endif

  return trackingResult;
}

#else

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(
    TrackingResult trackingResult)
{
  boost::optional<boost::timer::cpu_timer> relocalizationTimer;
  if (m_timeRelocalizer)
  relocalizationTimer = boost::timer::cpu_timer();

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

  const bool saveKeyframe = keyframeID >= 0;
  const bool performRelocalization = trackingResult
  == ITMTrackingState::TRACKING_FAILED && nearestNeighbour != -1;

  if (saveKeyframe)
  {
    // If the relocaliser added the current frame as a new keyframe, store its pose in the pose database.
    // Note that a new keyframe will only have been added if the tracking quality for this frame was good.
    m_poseDatabase->storePose(keyframeID, *trackingState->pose_d, 0);
  }
  else if (performRelocalization)
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
    m_refineTracker->TrackCamera(trackingState.get(), view.get());
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

  if (relocalizationTimer)
  {
    ORcudaSafeCall(cudaDeviceSynchronize());
    relocalizationTimer->stop();

    boost::timer::cpu_times elapsedTimes = relocalizationTimer->elapsed();

    if (performRelocalization)
    {
      m_relocalizationTimes.system += elapsedTimes.system;
      m_relocalizationTimes.wall += elapsedTimes.wall;
      m_relocalizationTimes.user += elapsedTimes.user;
      ++m_relocalizationCalls;
    }
    else
    {
      m_learningTimes.system += elapsedTimes.system;
      m_learningTimes.wall += elapsedTimes.wall;
      m_learningTimes.user += elapsedTimes.user;
      ++m_learningCalls;
    }
  }

  return trackingResult;
}

#endif

//#################### PROTECTED MEMBER FUNCTIONS ####################

void SLAMComponentWithScoreForest::compute_features(
    const ITMUChar4Image *inputRgbImage, const ITMFloatImage *inputDepthImage,
    const Vector4f &depthIntrinsics)
{
  Matrix4f identity;
  identity.setIdentity();

  compute_features(inputRgbImage, inputDepthImage, depthIntrinsics, identity);
}

void SLAMComponentWithScoreForest::compute_features(
    const ITMUChar4Image *inputRgbImage, const ITMFloatImage *inputDepthImage,
    const Vector4f &depthIntrinsics, const Matrix4f &invCameraPose)
{
#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6,
      "computing features on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
  m_featureExtractor->ComputeFeature(inputRgbImage, inputDepthImage,
      depthIntrinsics, m_featureImage.get(), invCameraPose);
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
