/**
 * spaint: SLAMComponentWithScoreForest.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include "ITMLib/Trackers/ITMTrackerFactory.h"
#include "ITMLib/Utils/ITMProjectionUtils.h"

#include <itmx/relocalisation/ICPRefiningRelocaliser.h>
#include <itmx/relocalisation/ICPRefiningRelocaliser.tpp>
#include <itmx/relocalisation/RelocaliserFactory.h>

#include "util/PosePersister.h"

#include "tvgutil/filesystem/PathFinder.h"
#include "tvgutil/misc/GlobalParameters.h"
#include "tvgutil/timing/TimeUtil.h"

#include <boost/lexical_cast.hpp>

#include <grove/relocalisation/ScoreRelocaliserFactory.h>

// Whether to enable VERBOSE timers, printing the time spent in each relocalisation phase, for each frame.
//#define ENABLE_VERBOSE_TIMERS

// Whether or not to save the modes associated to a certain set of forest leaves after the adaptation.
//#define SAVE_LEAF_MODES

// Whether or not to show the correspondences used to perform P-RANSAC.
//#define SHOW_RANSAC_CORRESPONDENCES

#ifdef SHOW_RANSAC_CORRESPONDENCES
#include <itmx/MemoryBlockFactory.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

#ifdef ENABLE_VERBOSE_TIMERS
#include <boost/timer/timer.hpp>
#endif

namespace bf = boost::filesystem;
using namespace InputSource;
using namespace ITMLib;
using namespace itmx;
using namespace ORUtils;
using namespace FernRelocLib;
using namespace grove;
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

SLAMComponentWithScoreForest::SLAMComponentWithScoreForest(const SLAMContext_Ptr &context,
                                                           const std::string &sceneID,
                                                           const ImageSourceEngine_Ptr &imageSourceEngine,
                                                           const std::string& trackerConfig,
                                                           MappingMode mappingMode,
                                                           TrackingMode trackingMode)
  : SLAMComponent(context, sceneID, imageSourceEngine, trackerConfig, mappingMode, trackingMode)
{
  const Settings_CPtr &settings = m_context->get_settings();

  const static std::string parametersNamespace = "SLAMComponentWithScoreForest.";
  const GlobalParameters &globalParams = GlobalParameters::instance();

  const SLAMState_Ptr &slamState = m_context->get_slam_state(m_sceneID);
  const Vector2i &depthImageSize = slamState->get_depth_image_size();
  const Vector2i &rgbImageSize = slamState->get_rgb_image_size();
  const SpaintVoxelScene_Ptr &voxelScene = slamState->get_voxel_scene();

  const std::string defaultRelocalisationForestPath =
      (bf::path(m_context->get_resources_dir()) / "DefaultRelocalizationForest.rf").string();
  m_relocalisationForestPath = globalParams.get_first_value<std::string>(
      parametersNamespace + "m_relocalisationForestPath", defaultRelocalisationForestPath);
  std::cout << "Loading relocalization forest from: " << m_relocalisationForestPath << '\n';

  m_updateRelocaliserEveryFrame =
      globalParams.get_first_value<bool>(parametersNamespace + "m_updateRelocaliserEveryFrame", true);
  m_relocaliseEveryFrame = globalParams.get_first_value<bool>(parametersNamespace + "m_relocaliseEveryFrame", false);

  // TODO: make this a proper member variable.
  const std::string m_relocaliserType =
      globalParams.get_first_value<std::string>(parametersNamespace + "m_relocaliserType", "forest");
  Relocaliser_Ptr nestedRelocaliser;
  if (m_relocaliserType == "forest")
  {
    nestedRelocaliser =
        ScoreRelocaliserFactory::make_score_relocaliser(settings->deviceType, m_relocalisationForestPath);
    //  nestedRelocaliser =
    //      ScoreRelocaliserFactory::make_score_relocaliser(ITMLibSettings::DEVICE_CPU, m_relocalisationForestPath);
  }
  else if (m_relocaliserType == "ferns")
  {
    if (m_relocaliseEveryFrame)
    {
      // Need to force the policy allowing the learning of new keyframes after relocalisation.
      nestedRelocaliser =
          RelocaliserFactory::make_custom_fern_relocaliser(depthImageSize,
                                                           settings->sceneParams.viewFrustum_min,
                                                           settings->sceneParams.viewFrustum_max,
                                                           FernRelocaliser::get_default_harvesting_threshold(),
                                                           FernRelocaliser::get_default_num_ferns(),
                                                           FernRelocaliser::get_default_num_decisions_per_fern(),
                                                           FernRelocaliser::ALWAYS_TRY_ADD);
    }
    else
    {
      nestedRelocaliser = RelocaliserFactory::make_default_fern_relocaliser(
          depthImageSize, settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max);
    }
  }
  else
  {
    throw std::invalid_argument("Invalid relocaliser type: " + m_relocaliserType);
  }

  // Refinement ICP tracker
  m_refinementTrackerParams =
      globalParams.get_first_value<std::string>(parametersNamespace + "m_refinementTrackerParams",
                                                "type=extended,levels=rrbb,minstep=1e-4,"
                                                "outlierSpaceC=0.1,outlierSpaceF=0.004,"
                                                "numiterC=20,numiterF=20,tukeyCutOff=8,"
                                                "framesToSkip=20,framesToWeight=50,failureDec=20.0");

  // Set up the refining relocaliser.
  m_refiningRelocaliser.reset(
      new ICPRefiningRelocaliser<SpaintVoxel, ITMVoxelIndex>(nestedRelocaliser,
                                                             voxelScene,
                                                             settings,
                                                             m_imageSourceEngine->getCalib(),
                                                             rgbImageSize,
                                                             depthImageSize,
                                                             m_refinementTrackerParams,
                                                             slamState->get_live_voxel_render_state()));

  m_refinementTracker.reset(ITMTrackerFactory::Instance().Make(settings->deviceType,
                                                               m_refinementTrackerParams.c_str(),
                                                               rgbImageSize,
                                                               depthImageSize,
                                                               m_lowLevelEngine.get(),
                                                               NULL,
                                                               voxelScene->sceneParams));
  m_refinementTrackingController.reset(new ITMTrackingController(m_refinementTracker.get(), settings.get()));

  m_timeRelocaliser = globalParams.get_first_value<bool>(parametersNamespace + "m_timeRelocaliser", false);
  m_learningCalls = 0;
  m_learningTimes.clear();
  m_relocalizationCalls = 0;
  m_relocalizationTimes.clear();

  m_saveRelocalisationPoses =
      globalParams.get_first_value<bool>(parametersNamespace + "m_saveRelocalisationPoses", false);

  if (m_saveRelocalisationPoses)
  {
    const std::string posesFolder = m_context->get_tag().empty() ? TimeUtil::get_iso_timestamp() : m_context->get_tag();

    m_sequentialPathGenerator.reset(SequentialPathGenerator(find_subdir_from_executable("reloc_poses") / posesFolder));

    std::cout << "Saving relocalization poses in: " << m_sequentialPathGenerator->get_base_dir() << std::endl;
    boost::filesystem::create_directories(m_sequentialPathGenerator->get_base_dir());
  }
}

//#################### DESTRUCTOR ####################
SLAMComponentWithScoreForest::~SLAMComponentWithScoreForest()
{
  if (m_timeRelocaliser)
  {
    std::cout << "Relocalizer called " << m_relocalizationCalls << "times.\n";

    if (m_relocalizationCalls > 0)
    {
      std::cout << "\tTotal time: " << boost::timer::format(m_relocalizationTimes, 6);
      // Average times
      m_relocalizationTimes.system /= m_relocalizationCalls;
      m_relocalizationTimes.user /= m_relocalizationCalls;
      m_relocalizationTimes.wall /= m_relocalizationCalls;
      std::cout << "\tAverage time: " << boost::timer::format(m_relocalizationTimes, 6);
    }

    std::cout << "Learner called " << m_learningCalls << "times.\n";
    if (m_learningCalls > 0)
    {
      std::cout << "\tTotal time: " << boost::timer::format(m_learningTimes, 6);
      // Average times
      m_learningTimes.system /= m_learningCalls;
      m_learningTimes.user /= m_learningCalls;
      m_learningTimes.wall /= m_learningCalls;
      std::cout << "\tAverage time: " << boost::timer::format(m_learningTimes, 6);
    }
  }
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(TrackingResult trackingResult)
{
  boost::optional<boost::timer::cpu_timer> relocalizationTimer;

  if (m_timeRelocaliser)
  {
    ORcudaSafeCall(cudaDeviceSynchronize());
    relocalizationTimer = boost::timer::cpu_timer();
  }

  const bool performRelocalization = m_relocaliseEveryFrame || trackingResult == ITMTrackingState::TRACKING_FAILED;
  const bool performLearning = m_relocaliseEveryFrame || trackingResult == ITMTrackingState::TRACKING_GOOD;

  const SLAMState_Ptr &slamState = m_context->get_slam_state(m_sceneID);
  const ITMFloatImage *inputDepthImage = slamState->get_view()->depth;
  const ITMUChar4Image *inputRGBImage = slamState->get_view()->rgb;

  const TrackingState_Ptr &trackingState = slamState->get_tracking_state();
  const SE3Pose trackedPose(*trackingState->pose_d);

  const View_Ptr &view = slamState->get_view();

  const Vector4f depthIntrinsics = view->calib.intrinsics_d.projectionParamsSimple.all;

  if (m_updateRelocaliserEveryFrame && !performLearning)
  {
#ifdef ENABLE_VERBOSE_TIMERS
    boost::timer::auto_cpu_timer t(6, "relocaliser update, overall: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_refiningRelocaliser->update();
  }

  if (performRelocalization)
  {
#ifdef ENABLE_VERBOSE_TIMERS
    boost::timer::auto_cpu_timer t(6, "relocalization, overall: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

#ifdef SAVE_LEAF_MODES
    {
      // Need to go through the ScoreRelocaliser interface.
      ScoreRelocaliser_Ptr scoreRelocaliser = boost::dynamic_pointer_cast<ScoreRelocaliser>(m_relocaliser);
      // Leaf indices selected randomly during the forest conversion step
      std::vector<uint32_t> predictionIndices{5198, 447, 5438, 7355, 1649};

      //    std::vector<uint32_t> predictionIndices
      //    { 5198, 447, 5438, 1664, 4753 };

      // For each prediction print centroids, covariances, nbInliers
      for (uint32_t treeIdx = 0; treeIdx < predictionIndices.size(); ++treeIdx)
      {
        const ScorePrediction p = scoreRelocaliser->get_raw_prediction(treeIdx, predictionIndices[treeIdx]);
        std::cout << p.nbClusters << ' ' << predictionIndices[treeIdx] << '\n';
        for (int modeIdx = 0; modeIdx < p.nbClusters; ++modeIdx)
        {
          const Mode3DColour &m = p.clusters[modeIdx];
          std::cout << m.nbInliers << ' ' << m.position.x << ' ' << m.position.y << ' ' << m.position.z << ' ';

          // Invert and transpose the covariance to print it in row-major format.
          Matrix3f posCovariance;
          m.positionInvCovariance.inv(posCovariance);
          posCovariance = posCovariance.t();

          for (int i = 0; i < 9; ++i) std::cout << posCovariance.m[i] << ' ';
          std::cout << '\n';
        }
        std::cout << '\n';
      }

      // Done for this test
      exit(0);
    }
#endif

    RefiningRelocaliser::RefinementDetails relocalisationDetails;
    boost::optional<ORUtils::SE3Pose> relocalisedPose =
        m_refiningRelocaliser->relocalise(inputRGBImage, inputDepthImage, depthIntrinsics, relocalisationDetails);

    if (relocalisedPose)
    {
      trackingState->pose_d->SetFrom(relocalisedPose.get_ptr());
      trackingResult = relocalisationDetails.refinementResult;

//      VoxelRenderState_Ptr liveVoxelRenderState = slamState->get_live_voxel_render_state();
//      SpaintVoxelScene_Ptr voxelScene = slamState->get_voxel_scene();
//      const bool resetVisibleList = true;
//      m_denseVoxelMapper->UpdateVisibleList(
//          view.get(), trackingState.get(), voxelScene.get(), liveVoxelRenderState.get(), resetVisibleList);
//      m_refinementTrackingController->Prepare(trackingState.get(),
//                                              voxelScene.get(),
//                                              view.get(),
//                                              m_context->get_voxel_visualisation_engine().get(),
//                                              liveVoxelRenderState.get());
//      m_refinementTrackingController->Track(trackingState.get(), view.get());
//      trackingResult = trackingState->trackerResult;

      if (m_saveRelocalisationPoses)
      {
        // Save poses
        PosePersister::save_pose_on_thread(relocalisationDetails.initialPose->GetInvM(),
                                           m_sequentialPathGenerator->make_path("pose-%06i.reloc.txt"));
        PosePersister::save_pose_on_thread(relocalisedPose->GetInvM(),
                                           m_sequentialPathGenerator->make_path("pose-%06i.icp.txt"));

        const Matrix4f final_pose = trackingResult == ITMTrackingState::TRACKING_GOOD
                                        ? relocalisedPose->GetInvM()
                                        : relocalisationDetails.initialPose->GetInvM();
        PosePersister::save_pose_on_thread(final_pose, m_sequentialPathGenerator->make_path("pose-%06i.final.txt"));
      }

#ifdef SHOW_RANSAC_CORRESPONDENCES
      static int relocalisationCount = 0;

      // Will need dynamic cast from the relocaliser type to get the best poses and read the first
      if (relocalisationCount++ == 451)
      {
        // Need to go through the ScoreRelocaliser interface.
        ScoreRelocaliser_Ptr scoreRelocaliser = boost::dynamic_pointer_cast<ScoreRelocaliser>(m_relocaliser);

        // Render RGB
        ITMUChar4Image_Ptr renderedRGB = MemoryBlockFactory::instance().make_image<Vector4u>(Vector2i(640, 480));

        m_context->get_visualisation_generator()->get_rgb_input(renderedRGB, view);

        cv::Mat outRGB = cv::Mat(480, 640, CV_8UC4, renderedRGB->GetData(MEMORYDEVICE_CPU)).clone();
        cv::cvtColor(outRGB, outRGB, CV_RGBA2BGR);

        // Get last pose candidates
        std::vector<PoseCandidate> candidates;
        scoreRelocaliser->get_best_poses(candidates);

        ITMUChar4Image_Ptr rendered = MemoryBlockFactory::instance().make_image<Vector4u>(Vector2i(640, 480));

        std::vector<cv::Mat> rgbWithPoints;
        std::vector<cv::Mat> raycastedPoses;

        std::vector<cv::Scalar> colours{CV_RGB(255, 0, 0), CV_RGB(0, 255, 0), CV_RGB(0, 0, 255)};

        for (size_t candidateIdx = 0; candidateIdx < candidates.size(); ++candidateIdx)
        {
          PoseCandidate &candidate = candidates[candidateIdx];

          ORUtils::SE3Pose pose;
          pose.SetInvM(candidate.cameraPose);

          m_context->get_visualisation_generator()->generate_voxel_visualisation(
              rendered,
              voxelScene,
              pose,
              view,
              liveVoxelRenderState,
              VisualisationGenerator::VT_SCENE_SEMANTICLAMBERTIAN);

          cv::Mat raycastedPose = cv::Mat(480, 640, CV_8UC4, rendered->GetData(MEMORYDEVICE_CPU)).clone();
          cv::cvtColor(raycastedPose, raycastedPose, CV_RGBA2BGR);

          // Draw kabsch points in the images
          cv::Mat rgbKabsch = outRGB.clone();
          for (size_t i = 0; i < 3; ++i)
          {

            // Draw camera point
            Vector2f ptCamera = project(candidate.pointsCamera[i], depthIntrinsics);
            cv::circle(rgbKabsch, cv::Point(ptCamera.x, ptCamera.y), 12, cv::Scalar::all(255), CV_FILLED);
            cv::circle(rgbKabsch, cv::Point(ptCamera.x, ptCamera.y), 9, colours[i], CV_FILLED);

            // Draw world point
            Vector2f ptRaycast = project(pose.GetM() * candidate.pointsWorld[i], depthIntrinsics);
            cv::circle(raycastedPose, cv::Point(ptRaycast.x, ptRaycast.y), 12, cv::Scalar::all(255), CV_FILLED);
            cv::circle(raycastedPose, cv::Point(ptRaycast.x, ptRaycast.y), 9, colours[i], CV_FILLED);
          }

          rgbWithPoints.push_back(rgbKabsch);
          raycastedPoses.push_back(raycastedPose);
        }

        const int textHeight = 10;
        const int gap = 10;
        const int blockWidth = 640 + gap;
        const int blockHeight = 480 * 2 + textHeight;

        cv::Mat outCanvas(blockHeight * 4 - textHeight, blockWidth * 4 - gap, CV_8UC3, cv::Scalar::all(255));

        //        for (size_t i = 0; i < rgbWithPoints.size(); ++i)
        for (size_t i = 0; i < 16; ++i)
        {
          cv::Rect rgbRect(blockWidth * (i % 4), blockHeight * (i / 4), 640, 480);
          cv::Rect raycastRect(blockWidth * (i % 4), blockHeight * (i / 4) + 480, 640, 480);

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
          std::string outPath = m_sequentialPathGenerator->make_path("candidates-%06i.png").string();
          cv::imwrite(outPath, outCanvas);
        }

#if 0
        // Print the modes associated to candidate 0.
        PoseCandidate candidate = candidates[0];

        Vector2f ptCamera[3];
        Vector2i ptCameraInt[3];
        int linearIdxDownsampled[3];
        ScorePrediction predictions[3];
        for (int i = 0; i < 3; ++i)
        {
          ptCamera[i] = project(candidate.pointsCamera[i], depthIntrinsics);
          ptCameraInt[i] = ptCamera[i].toInt();
          linearIdxDownsampled[i] = (inputDepthImage->noDims.width / 4) * (ptCameraInt[i].y / 4) + ptCameraInt[i].x / 4;
          predictions[i] =
              scoreRelocaliser->get_predictions_image()->GetData(MEMORYDEVICE_CPU)[linearIdxDownsampled[i]];
        }

        for (int i = 0; i < 3; ++i)
        {
          const ScorePrediction p = predictions[i];
          std::cout << p.nbClusters << ' ' << linearIdxDownsampled[i] << '\n';
          for (int modeIdx = 0; modeIdx < p.nbClusters; ++modeIdx)
          {
            const Mode3DColour &m = p.clusters[modeIdx];
            std::cout << 1 << ' ' << m.position.x << ' ' << m.position.y << ' ' << m.position.z << ' ';

            // Invet and transpose the covariance to print it in row-major
            Matrix3f posCovariance;
            m.positionInvCovariance.inv(posCovariance);
            posCovariance = posCovariance.t();

            for (int i = 0; i < 9; ++i) std::cout << posCovariance.m[i] << ' ';
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
        // Prevent fusion from happening to avoid integrating new data in the map, since we are in evaluation mode.
        trackingResult = ITMTrackingState::TRACKING_POOR;
      }
    }
    else
    {
      std::cout << "Cannot estimate a pose candidate." << std::endl;

      if (m_saveRelocalisationPoses)
      {
        // Save dummy poses
        Matrix4f invalid_pose;
        invalid_pose.setValues(std::numeric_limits<float>::quiet_NaN());

        PosePersister::save_pose_on_thread(invalid_pose, m_sequentialPathGenerator->make_path("pose-%06i.reloc.txt"));
        PosePersister::save_pose_on_thread(invalid_pose, m_sequentialPathGenerator->make_path("pose-%06i.icp.txt"));
        PosePersister::save_pose_on_thread(invalid_pose, m_sequentialPathGenerator->make_path("pose-%06i.final.txt"));
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
#ifdef ENABLE_VERBOSE_TIMERS
    boost::timer::auto_cpu_timer t(6, "relocaliser, integration: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    m_refiningRelocaliser->integrate_rgbd_pose_pair(inputRGBImage, inputDepthImage, depthIntrinsics, trackedPose);

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

  if (m_relocaliseEveryFrame)
  {
    // Restore tracked pose
    trackingState->pose_d->SetFrom(&trackedPose);
    // The assumption is that we are using the ground truth trajectory so the tracker always succeeds.
    trackingResult = ITMTrackingState::TRACKING_GOOD;
  }

  return trackingResult;
}

} // namespace spaint
