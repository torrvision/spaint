/**
 * spaint: SLAMComponentWithScoreForest.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include "ITMLib/Trackers/ITMTrackerFactory.h"
#include "ITMLib/Utils/ITMProjectionUtils.h"

#include "util/PosePersister.h"

#include "tvgutil/filesystem/PathFinder.h"
#include "tvgutil/misc/GlobalParameters.h"
#include "tvgutil/timing/TimeUtil.h"

#include <boost/lexical_cast.hpp>

#include <grove/relocalisation/RelocaliserFactory.h>

//#define ENABLE_TIMERS
//#define VISUALIZE_INLIERS
//#define SAVE_LEAF_MODES
//#define SAVE_INLIERS
//#define USE_FERN_RELOCALISER
//#define SHOW_RANSAC_CORRESPONDENCES

#ifdef ENABLE_TIMERS
#include <boost/timer/timer.hpp>
#endif

#ifdef VISUALIZE_INLIERS
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif

namespace bf = boost::filesystem;
using namespace InputSource;
using namespace ITMLib;
using namespace ORUtils;
using namespace FernRelocLib;
using namespace grove;
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

  const static std::string parametersNamespace = "SLAMComponentWithScoreForest.";
  const GlobalParameters& globalParams = GlobalParameters::instance();

  const std::string defaultRelocalisationForestPath = (bf::path(m_context->get_resources_dir()) / "DefaultRelocalizationForest.rf").string();
  m_relocalisationForestPath = globalParams.get_typed_value<std::string>(parametersNamespace + "m_relocalisationForestPath", defaultRelocalisationForestPath);
  std::cout << "Loading relocalization forest from: " << m_relocalisationForestPath << '\n';

  m_updateForestModesEveryFrame = true;

  m_scoreRelocaliser = RelocaliserFactory::make_score_relocaliser(settings->deviceType, m_relocalisationForestPath);
//  m_scoreRelocaliser = RelocaliserFactory::make_score_relocaliser(ITMLibSettings::DEVICE_CPU, m_relocalisationForestPath);

  // Refinement ICP tracker
  const SLAMState_Ptr& slamState = m_context->get_slam_state(m_sceneID);
  const Vector2i& depthImageSize = slamState->get_depth_image_size();
  const Vector2i& rgbImageSize = slamState->get_rgb_image_size();
  const SpaintVoxelScene_Ptr& voxelScene = slamState->get_voxel_scene();

  m_refinementTrackerParams =
      globalParams.get_typed_value<std::string>(
          parametersNamespace + "m_refinementTrackerParams",
          "type=extended,levels=rrbb,minstep=1e-4,"
          "outlierSpaceC=0.1,outlierSpaceF=0.004,"
          "numiterC=20,numiterF=20,tukeyCutOff=8,"
          "framesToSkip=20,framesToWeight=50,failureDec=20.0"
      );

  m_refinementTracker.reset(
      ITMTrackerFactory::Instance().Make(settings->deviceType, m_refinementTrackerParams.c_str(), rgbImageSize,
          depthImageSize, m_lowLevelEngine.get(), NULL,
          voxelScene->sceneParams));
  m_refinementTrackingController.reset(
      new ITMTrackingController(m_refinementTracker.get(), settings.get()));

  m_timeRelocalizer = true;
  m_learningCalls = 0;
  m_learningTimes.clear();
  m_relocalizationCalls = 0;
  m_relocalizationTimes.clear();

  m_relocaliseAfterEveryFrame = globalParams.get_typed_value<bool>(
      parametersNamespace + "m_relocaliseAfterEveryFrame", false);
  m_saveRelocalisationPoses = globalParams.get_typed_value<bool>(
      parametersNamespace + "m_saveRelocalisationPoses", false);

  if (m_saveRelocalisationPoses)
  {
    const std::string posesFolder =
        m_context->get_tag().empty() ?
            TimeUtil::get_iso_timestamp() : m_context->get_tag();

    m_sequentialPathGenerator.reset(
        SequentialPathGenerator(
            find_subdir_from_executable("reloc_poses") / posesFolder));

    std::cout << "Saving relocalization poses in: "
        << m_sequentialPathGenerator->get_base_dir() << std::endl;
    boost::filesystem::create_directories(
        m_sequentialPathGenerator->get_base_dir());
  }
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

  const bool performRelocalization = m_relocaliseAfterEveryFrame
      || trackingResult == ITMTrackingState::TRACKING_FAILED;
  const bool performLearning = m_relocaliseAfterEveryFrame
      || trackingResult == ITMTrackingState::TRACKING_GOOD;

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
    m_scoreRelocaliser->idle_update();
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
      const ScorePrediction p = m_relocalisationForest->get_prediction(treeIdx,
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

    boost::optional<PoseCandidate> pose_candidate = m_scoreRelocaliser->estimate_pose(inputRGBImage, inputDepthImage, depthIntrinsics);

    if (pose_candidate)
    {
#ifdef VISUALIZE_INLIERS
      cv::Mat inliers = cv::Mat::zeros(
          cv::Size(m_rgbdPatchDescriptorImage->noDims.width, m_rgbdPatchDescriptorImage->noDims.height),
          CV_32FC1);
      inliers.setTo(std::numeric_limits<float>::quiet_NaN());

      for (int i = 0; i < pose_candidate->nbInliers; ++i)
      {
        int idx = pose_candidate->inliers[i].linearIdx;
        float energy = pose_candidate->inliers[i].energy;

        int x = idx % m_rgbdPatchDescriptorImage->noDims.width;
        int y = idx / m_rgbdPatchDescriptorImage->noDims.width;

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

      VoxelRenderState_Ptr liveVoxelRenderState = slamState->get_live_voxel_render_state();
      SpaintVoxelScene_Ptr voxelScene = slamState->get_voxel_scene();
      const bool resetVisibleList = true;
      m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(),
          voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
      m_refinementTrackingController->Prepare(trackingState.get(),
          voxelScene.get(), view.get(),
          m_context->get_voxel_visualisation_engine().get(),
          liveVoxelRenderState.get());
      m_refinementTrackingController->Track(trackingState.get(), view.get());
      trackingResult = trackingState->trackerResult;

//      std::cout << "Refinement: "
//          << (trackingState->trackerResult == TrackingResult::TRACKING_GOOD ?
//              "SUCCESS" : "FAIL") << "\n Refined pose:\n"
//          << trackingState->pose_d->GetInvM() << std::endl;

      if (m_saveRelocalisationPoses && m_sequentialPathGenerator)
      {
        // Save poses
        PosePersister::save_pose_on_thread(pose_candidate->cameraPose,
            m_sequentialPathGenerator->make_path("pose-%06i.reloc.txt"));
        PosePersister::save_pose_on_thread(trackingState->pose_d->GetInvM(),
            m_sequentialPathGenerator->make_path("pose-%06i.icp.txt"));

        const Matrix4f final_pose =
            trackingResult == ITMTrackingState::TRACKING_GOOD ?
                trackingState->pose_d->GetInvM() : pose_candidate->cameraPose;

        PosePersister::save_pose_on_thread(final_pose,
            m_sequentialPathGenerator->make_path("pose-%06i.final.txt"));
      }

#ifdef SHOW_RANSAC_CORRESPONDENCES
      static int relocalisationCount = 0;
      if (relocalisationCount++ == 451)
      {
        // Render RGB
        ITMUChar4Image_Ptr renderedRGB =
        MemoryBlockFactory::instance().make_image<Vector4u>(
            Vector2i(640, 480));

        m_context->get_visualisation_generator()->get_rgb_input(renderedRGB,
            view);

        cv::Mat outRGB = cv::Mat(480, 640, CV_8UC4,
            renderedRGB->GetData(MEMORYDEVICE_CPU)).clone();
        cv::cvtColor(outRGB, outRGB, CV_RGBA2BGR);

        // Get last pose candidates
        std::vector<PoseCandidate> candidates;
        m_preemptiveRansac->get_best_poses(candidates);

        ITMUChar4Image_Ptr rendered = MemoryBlockFactory::instance().make_image<
        Vector4u>(Vector2i(640, 480));

        std::vector<cv::Mat> rgbWithPoints;
        std::vector<cv::Mat> raycastedPoses;

        std::vector<cv::Scalar> colours
        { CV_RGB(255, 0, 0), CV_RGB(0, 255, 0), CV_RGB(0, 0, 255)};

        for (size_t candidateIdx = 0; candidateIdx < candidates.size();
            ++candidateIdx)
        {
          PoseCandidate &candidate = candidates[candidateIdx];

          ORUtils::SE3Pose pose;
          pose.SetInvM(candidate.cameraPose);

          m_context->get_visualisation_generator()->generate_voxel_visualisation(
              rendered, voxelScene, pose, view, liveVoxelRenderState,
              VisualisationGenerator::VT_SCENE_SEMANTICLAMBERTIAN);

          cv::Mat raycastedPose = cv::Mat(480, 640, CV_8UC4,
              rendered->GetData(MEMORYDEVICE_CPU)).clone();
          cv::cvtColor(raycastedPose, raycastedPose, CV_RGBA2BGR);

          // Draw kabsch points in the images
          cv::Mat rgbKabsch = outRGB.clone();
          for (size_t i = 0; i < 3; ++i)
          {

            // Draw camera point
            Vector2f ptCamera = project(candidate.cameraPoints[i],
                depthIntrinsics);
            cv::circle(rgbKabsch, cv::Point(ptCamera.x, ptCamera.y), 12,
                cv::Scalar::all(255), CV_FILLED);
            cv::circle(rgbKabsch, cv::Point(ptCamera.x, ptCamera.y), 9,
                colours[i],
                CV_FILLED);

            // Draw world point
            Vector2f ptRaycast = project(pose.GetM() * candidate.worldPoints[i],
                depthIntrinsics);
            cv::circle(raycastedPose, cv::Point(ptRaycast.x, ptRaycast.y), 12,
                cv::Scalar::all(255), CV_FILLED);
            cv::circle(raycastedPose, cv::Point(ptRaycast.x, ptRaycast.y), 9,
                colours[i],
                CV_FILLED);
          }

          rgbWithPoints.push_back(rgbKabsch);
          raycastedPoses.push_back(raycastedPose);
        }

        const int textHeight = 10;
        const int gap = 10;
        const int blockWidth = 640 + gap;
        const int blockHeight = 480 * 2 + textHeight;

        cv::Mat outCanvas(blockHeight * 4 - textHeight, blockWidth * 4 - gap,
            CV_8UC3, cv::Scalar::all(255));

//        for (size_t i = 0; i < rgbWithPoints.size(); ++i)
        for (size_t i = 0; i < 16; ++i)
        {
          cv::Rect rgbRect(blockWidth * (i % 4), blockHeight * (i / 4), 640,
              480);
          cv::Rect raycastRect(blockWidth * (i % 4),
              blockHeight * (i / 4) + 480, 640, 480);

//          std::string text = boost::lexical_cast<std::string>(
//              candidates[i].energy);
//          int baseline;
//          cv::Size textSize = cv::getTextSize(text, CV_FONT_HERSHEY_SIMPLEX,
//              1.1, 4, &baseline);
//
//          cv::Mat textMat(textHeight, 640, CV_8UC3, cv::Scalar::all(255));
//
//          cv::Point textOrg((textMat.cols - textSize.width) / 2,
//              (textMat.rows + textSize.height) / 2);
//
//          cv::putText(textMat, text, textOrg,
//          CV_FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar::all(0), 4, cv::LINE_AA);
//
//          cv::Rect textRect(blockWidth * (i % 8), blockHeight * (i / 8) + 960,
//              640, textHeight);

          rgbWithPoints[i].copyTo(outCanvas(rgbRect));
          raycastedPoses[i].copyTo(outCanvas(raycastRect));
//          textMat.copyTo(outCanvas(textRect));
        }

#if 0
        cv::namedWindow("Canvas", CV_WINDOW_KEEPRATIO);
        cv::imshow("Canvas", outCanvas);
        cv::waitKey();
#endif

        if (m_sequentialPathGenerator)
        {
          std::string outPath = m_sequentialPathGenerator->make_path(
              "candidates-%06i.png").string();
          cv::imwrite(outPath, outCanvas);
        }

#if 0
        // Print candidate 0 modes
        PoseCandidate candidate = candidates[0];

        Vector2f ptCamera[3];
        Vector2i ptCameraInt[3];
        int linearIdxDownsampled[3];
        ScorePrediction predictions[3];
        for (int i = 0; i < 3; ++i)
        {
          ptCamera[i] = project(candidate.cameraPoints[i], depthIntrinsics);
          ptCameraInt[i] = ptCamera[i].toInt();
          linearIdxDownsampled[i] = (inputDepthImage->noDims.width / 4)
          * (ptCameraInt[i].y / 4) + ptCameraInt[i].x / 4;
          predictions[i] =
          m_predictionsImage->GetData(MEMORYDEVICE_CPU)[linearIdxDownsampled[i]];
        }

        for (int i = 0; i < 3; ++i)
        {
          const ScorePrediction p = predictions[i];
          std::cout << p.nbModes << ' ' << linearIdxDownsampled[i] << '\n';
          for (int modeIdx = 0; modeIdx < p.nbModes; ++modeIdx)
          {
            const ScoreMode &m = p.modes[modeIdx];
            std::cout << 1 << ' ' << m.position.x << ' ' << m.position.y << ' '
            << m.position.z << ' ';

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

        exit(0);
#endif
      }

#endif

      if (m_saveRelocalisationPoses)
      {
        trackingResult = ITMTrackingState::TRACKING_POOR;
      }
    }
    else
    {
      std::cout << "Cannot estimate a pose candidate." << std::endl;

      if (m_saveRelocalisationPoses && m_sequentialPathGenerator)
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
//#ifdef ENABLE_TIMERS
//    boost::timer::auto_cpu_timer t(6,
//        "add features to forest: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//#endif

    const Matrix4f invCameraPose = trackedPose.GetInvM();
    m_scoreRelocaliser->integrate_measurements(inputRGBImage, inputDepthImage, depthIntrinsics, invCameraPose);

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

  if (m_relocaliseAfterEveryFrame)
  {
    // Restore tracked pose
    trackingState->pose_d->SetFrom(&trackedPose);
    trackingResult = ITMTrackingState::TRACKING_GOOD; // The assumption is that we are using the ground truth trajectory.
  }

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
  const SE3Pose trackedPose(*trackingState->pose_d);

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
  const bool performRelocalization = m_relocaliseAfterEveryFrame
  || trackingResult == ITMTrackingState::TRACKING_FAILED;

  if (performRelocalization)
  {
    // If the tracking failed but a nearest keyframe was found by the relocaliser, reset the pose to that
    // of the keyframe and rerun the tracker for this frame.
    ORUtils::SE3Pose relocPose(std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN());
    ORUtils::SE3Pose refinedPose = relocPose;

    if (nearestNeighbour != -1)
    {
      relocPose = m_poseDatabase->retrievePose(nearestNeighbour).pose;

      trackingState->pose_d->SetFrom(&relocPose);

      const bool resetVisibleList = true;
      m_denseVoxelMapper->UpdateVisibleList(view.get(), trackingState.get(),
          voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
      m_refinementTrackingController->Prepare(trackingState.get(),
          voxelScene.get(), view.get(),
          m_context->get_voxel_visualisation_engine().get(),
          liveVoxelRenderState.get());
      m_refinementTrackingController->Track(trackingState.get(), view.get());
      trackingResult = trackingState->trackerResult;
      refinedPose = *trackingState->pose_d;
    }

    if (m_saveRelocalisationPoses && m_sequentialPathGenerator)
    {
      // Save poses
      PosePersister::save_pose_on_thread(relocPose.GetInvM(),
          m_sequentialPathGenerator->make_path("pose-%06i.reloc.txt"));
      PosePersister::save_pose_on_thread(refinedPose.GetInvM(),
          m_sequentialPathGenerator->make_path("pose-%06i.icp.txt"));

      const Matrix4f final_pose =
      trackingResult == TrackingResult::TRACKING_GOOD ?
      refinedPose.GetInvM() : relocPose.GetInvM();

      PosePersister::save_pose_on_thread(final_pose,
          m_sequentialPathGenerator->make_path("pose-%06i.final.txt"));

      m_sequentialPathGenerator->increment_index();
    }

    if (m_saveRelocalisationPoses)
    {
      trackingResult = TrackingResult::TRACKING_POOR;
    }

    if (!m_relocaliseAfterEveryFrame)
    {
      // Set the number of frames for which the tracking quality must be good before the relocaliser can consider
      // adding a new keyframe.
      m_keyframeDelay = 10;
    }
  }

  if (saveKeyframe)
  {
    // If the relocaliser added the current frame as a new keyframe, store its pose in the pose database.
    // Note that a new keyframe will only have been added if the tracking quality for this frame was good.
    m_poseDatabase->storePose(keyframeID, trackedPose, 0);
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

  if (m_relocaliseAfterEveryFrame)
  {
    // Restore tracked pose
    trackingState->pose_d->SetFrom(&trackedPose);
    trackingResult = TrackingResult::TRACKING_GOOD;
  }

  return trackingResult;
}

#endif

}
