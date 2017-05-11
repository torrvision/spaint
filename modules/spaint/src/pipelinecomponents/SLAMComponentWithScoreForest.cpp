/**
 * spaint: SLAMComponentWithScoreForest.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMComponentWithScoreForest.h"

#include "ITMLib/Trackers/ITMTrackerFactory.h"
#include "ITMLib/Utils/ITMProjectionUtils.h"

#include <itmx/PosePersister.h>
#include <itmx/relocalisation/ICPRefiningRelocaliser.h>
#include <itmx/relocalisation/ICPRefiningRelocaliser.tpp>
#include <itmx/relocalisation/RelocaliserFactory.h>

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
                                                           const std::string &trackerConfig,
                                                           MappingMode mappingMode,
                                                           TrackingMode trackingMode)
  : SLAMComponent(context, sceneID, imageSourceEngine, trackerConfig, mappingMode, trackingMode)
{
}

//#################### DESTRUCTOR ####################
SLAMComponentWithScoreForest::~SLAMComponentWithScoreForest() {}

//#################### PROTECTED MEMBER FUNCTIONS ####################

SLAMComponent::TrackingResult SLAMComponentWithScoreForest::process_relocalisation(TrackingResult trackingResult)
{
  const bool performRelocalization = m_relocaliseEveryFrame || trackingResult == ITMTrackingState::TRACKING_FAILED;
  const bool performLearning = m_relocaliseEveryFrame || trackingResult == ITMTrackingState::TRACKING_GOOD;

  const RefiningRelocaliser_Ptr &relocaliser = m_context->get_relocaliser(m_sceneID);
  const SLAMState_Ptr &slamState = m_context->get_slam_state(m_sceneID);
  const ITMFloatImage *inputDepthImage = slamState->get_view()->depth;
  const ITMUChar4Image *inputRGBImage = slamState->get_view()->rgb;

  const TrackingState_Ptr &trackingState = slamState->get_tracking_state();
  const SE3Pose trackedPose(*trackingState->pose_d);

  const View_Ptr &view = slamState->get_view();

  const Vector4f depthIntrinsics = view->calib.intrinsics_d.projectionParamsSimple.all;

  if (m_relocaliserUpdateEveryFrame && !performLearning)
  {
#ifdef ENABLE_VERBOSE_TIMERS
    boost::timer::auto_cpu_timer t(6, "relocaliser update, overall: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    relocaliser->update();
  }

  if (performRelocalization)
  {
#ifdef ENABLE_VERBOSE_TIMERS
    boost::timer::auto_cpu_timer t(6, "relocalization, overall: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

#ifdef SAVE_LEAF_MODES
    {
      // Need to go through the ScoreRelocaliser interface.
      ScoreRelocaliser_Ptr scoreRelocaliser =
          boost::dynamic_pointer_cast<ScoreRelocaliser>(relocaliser->get_inner_relocaliser());
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
        relocaliser->relocalise(inputRGBImage, inputDepthImage, depthIntrinsics, relocalisationDetails);

    if (relocalisedPose)
    {
      trackingState->pose_d->SetFrom(relocalisedPose.get_ptr());

      if (relocalisationDetails.refinementResult == TrackingResult::TRACKING_GOOD)
      {
        static int count = 0;
        std::cout << "Refinement result: GOOD - " << ++count << '\n';
      }
      else if (relocalisationDetails.refinementResult == TrackingResult::TRACKING_FAILED)
      {
        static int count = 0;
        std::cout << "Refinement result: FAIL - " << ++count << '\n';
      }

      trackingResult = relocalisationDetails.refinementResult;

#ifdef SHOW_RANSAC_CORRESPONDENCES
      static int relocalisationCount = 0;

      // Will need dynamic cast from the relocaliser type to get the best poses and read the first
      if (relocalisationCount++ == 451)
      {
        // Need to go through the ScoreRelocaliser interface.
        ScoreRelocaliser_Ptr scoreRelocaliser =
            boost::dynamic_pointer_cast<ScoreRelocaliser>(relocaliser->get_inner_relocaliser());

        // Need to have the scene and renderState available.
        VoxelRenderState_Ptr liveVoxelRenderState = slamState->get_live_voxel_render_state();
        SpaintVoxelScene_Ptr voxelScene = slamState->get_voxel_scene();

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
    }
    else
    {
      std::cout << "Cannot estimate a pose candidate." << std::endl;
    }
  }

  if (performLearning)
  {
#ifdef ENABLE_VERBOSE_TIMERS
    boost::timer::auto_cpu_timer t(6, "relocaliser, integration: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    relocaliser->integrate_rgbd_pose_pair(inputRGBImage, inputDepthImage, depthIntrinsics, trackedPose);
  }

  if (m_relocaliseEveryFrame)
  {
    // Restore tracked pose.
    trackingState->pose_d->SetFrom(&trackedPose);
    // The assumption is that we are using the ground truth trajectory so the tracker always succeeds.
    trackingResult = ITMTrackingState::TRACKING_GOOD;
  }

  return trackingResult;
}

} // namespace spaint
