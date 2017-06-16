/**
 * spaintgui: RelocaliserFiguresGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "RelocaliserFiguresGenerator.h"

#include <fstream>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <opencv2/opencv.hpp>

#include <ITMLib/Utils/ITMProjectionUtils.h>

#include <itmx/base/MemoryBlockFactory.h>
#include <itmx/relocalisation/RefiningRelocaliser.h>
#include <itmx/relocalisation/Relocaliser.h>
using namespace itmx;

#include <grove/relocalisation/interface/ScoreRelocaliser.h>
using namespace grove;

#include <tvgutil/filesystem/SequentialPathGenerator.h>
using namespace tvgutil;

using namespace spaint;

namespace spaintgui {

void RelocaliserFiguresGenerator::show_growing_leaf_modes(const Model_Ptr &model)
{
  static uint32_t frameIdx = 0;

  if(frameIdx++ % 15 != 0) return; // Work every N frames
//  if(frameIdx > 1031) exit(0);     // done

  static SequentialPathGenerator pathGenerator("./clusters");
  fs::create_directories(pathGenerator.get_base_dir());

  const std::string sceneId = model->get_world_scene_id();
  Relocaliser_CPtr relocaliser = model->get_relocaliser(sceneId);

  // Need to go through the ScoreRelocaliser interface.
  ScoreRelocaliser_CPtr scoreRelocaliser = boost::dynamic_pointer_cast<const ScoreRelocaliser>(
      boost::dynamic_pointer_cast<const RefiningRelocaliser>(relocaliser)->get_inner_relocaliser());

  std::vector<uint32_t> predictionIndices{3234, 4335, 4545, 6565, 6666};

  // Save cluster contents.
  {
    const std::string fileName =
        pathGenerator
            .make_path(model->get_settings()->get_first_value<std::string>("experimentTag", "predictionClusters") +
                       "_%04d.txt")
            .string();

    std::cout << "Saving clusters in " << fileName << '\n';

    std::ofstream outFile(fileName);

    // For each prediction print centroids, covariances, nbInliers
    for(uint32_t treeIdx = 0; treeIdx < predictionIndices.size(); ++treeIdx)
    {
      const ScorePrediction p = scoreRelocaliser->get_raw_prediction(treeIdx, predictionIndices[treeIdx]);
      outFile << p.nbClusters << ' ' << predictionIndices[treeIdx] << '\n';
      for(int modeIdx = 0; modeIdx < p.nbClusters; ++modeIdx)
      {
        const Mode3DColour &m = p.clusters[modeIdx];
        outFile << m.nbInliers << ' ' << m.position.x << ' ' << m.position.y << ' ' << m.position.z << ' ';

        // Invert and transpose the covariance to print it in row-major format.
        Matrix3f posCovariance;
        m.positionInvCovariance.inv(posCovariance);
        posCovariance = posCovariance.t();

        for(int i = 0; i < 9; ++i) outFile << posCovariance.m[i] << ' ';
        outFile << '\n';
      }
      outFile << '\n';
    }
  }

  // Save reservoir contents
  {
    const std::string fileName =
        pathGenerator
            .make_path(model->get_settings()->get_first_value<std::string>("experimentTag", "predictionClusters") +
                       "_reservoirs_%04d.txt")
            .string();

    std::cout << "Saving reservoir contents in " << fileName << '\n';

    std::ofstream outFile(fileName);

    // For each prediction print centroids, covariances, nbInliers
    for(uint32_t treeIdx = 0; treeIdx < predictionIndices.size(); ++treeIdx)
    {
      const std::vector<Keypoint3DColour> examples = scoreRelocaliser->get_reservoir_contents(treeIdx, predictionIndices[treeIdx]);

      outFile << examples.size() << ' ';

      for(size_t exampleIdx = 0; exampleIdx < examples.size(); ++exampleIdx)
      {
        const Keypoint3DColour &e = examples[exampleIdx];

        outFile << e.position.x << ' ' << e.position.y << ' ' << e.position.z << ' ';
      }
      outFile << '\n';
    }
  }

  pathGenerator.increment_index();
}

void RelocaliserFiguresGenerator::show_leaf_modes(const Model_Ptr &model)
{
  // The modes figure in the paper was obtained from the chess scene.
  // The figure showed the modes in a specific set of leaves after training, since the chess scene has 4000 training
  // frames, we run the rest of the method only after that number of frames has been processed.

  static uint32_t frameIdx = 0;
  //  if (frameIdx++ < 4000) return; // chess
  //  if (frameIdx++ < 715) return; // apt1-kitchen
  if(frameIdx++ < 1031) return; // office2-5a

  const std::string sceneId = model->get_world_scene_id();
  Relocaliser_CPtr relocaliser = model->get_relocaliser(sceneId);

  // Need to go through the ScoreRelocaliser interface.
  ScoreRelocaliser_CPtr scoreRelocaliser = boost::dynamic_pointer_cast<const ScoreRelocaliser>(
      boost::dynamic_pointer_cast<const RefiningRelocaliser>(relocaliser)->get_inner_relocaliser());

  // Leaf indices selected randomly during the forest conversion step (for chess)
  //  std::vector<uint32_t> predictionIndices{5198, 447, 5438, 7355, 1649};

  // Leaf indices for apt1-kitchen
  //  std::vector<uint32_t> predictionIndices{4242, 42, 4545, 5555, 6666};
  std::vector<uint32_t> predictionIndices{3234, 4335, 4545, 6565, 6666};

  //    std::vector<uint32_t> predictionIndices
  //    { 5198, 447, 5438, 1664, 4753 };

  // For each prediction print centroids, covariances, nbInliers
  for(uint32_t treeIdx = 0; treeIdx < predictionIndices.size(); ++treeIdx)
  {
    const ScorePrediction p = scoreRelocaliser->get_raw_prediction(treeIdx, predictionIndices[treeIdx]);
    std::cout << p.nbClusters << ' ' << predictionIndices[treeIdx] << '\n';
    for(int modeIdx = 0; modeIdx < p.nbClusters; ++modeIdx)
    {
      const Mode3DColour &m = p.clusters[modeIdx];
      std::cout << m.nbInliers << ' ' << m.position.x << ' ' << m.position.y << ' ' << m.position.z << ' ';

      // Invert and transpose the covariance to print it in row-major format.
      Matrix3f posCovariance;
      m.positionInvCovariance.inv(posCovariance);
      posCovariance = posCovariance.t();

      for(int i = 0; i < 9; ++i) std::cout << posCovariance.m[i] << ' ';
      std::cout << '\n';
    }
    std::cout << '\n';
  }

  // Done for this test
  exit(0);
}

void RelocaliserFiguresGenerator::show_ransac_correspondences(const Model_Ptr &model)
{
  // One of the figures in the paper was obtained from the stairs sequence, specifically from the 451th frame in that
  // sequence.
  // Since the training set has 2000 images we wait for 2451 frames before saving the images.
  static uint32_t frameIdx = 0;

  if(frameIdx++ <= 2451) return;

  const std::string sceneId = model->get_world_scene_id();

  // Useful variables.
  Relocaliser_CPtr relocaliser = model->get_relocaliser(sceneId);
  SLAMState_Ptr slamState = model->get_slam_state(sceneId);
  View_Ptr view = slamState->get_view();
  Vector4f depthIntrinsics = view->calib.intrinsics_d.projectionParamsSimple.all;

  // Will need dynamic cast from the relocaliser type to get the best poses and read the first
  ScoreRelocaliser_CPtr scoreRelocaliser = boost::dynamic_pointer_cast<const ScoreRelocaliser>(
      boost::dynamic_pointer_cast<const RefiningRelocaliser>(relocaliser)->get_inner_relocaliser());

  // Need to have the scene and renderState available.
  VoxelRenderState_Ptr liveVoxelRenderState = slamState->get_live_voxel_render_state();
  SpaintVoxelScene_Ptr voxelScene = slamState->get_voxel_scene();

  // Render RGB
  ITMUChar4Image_Ptr renderedRGB = MemoryBlockFactory::instance().make_image<Vector4u>(Vector2i(640, 480));

  model->get_visualisation_generator()->get_rgb_input(renderedRGB, view);

  cv::Mat outRGB = cv::Mat(480, 640, CV_8UC4, renderedRGB->GetData(MEMORYDEVICE_CPU)).clone();
  cv::cvtColor(outRGB, outRGB, CV_RGBA2BGR);

  // Get last pose candidates
  std::vector<PoseCandidate> candidates;
  scoreRelocaliser->get_best_poses(candidates);

  ITMUChar4Image_Ptr rendered = MemoryBlockFactory::instance().make_image<Vector4u>(Vector2i(640, 480));

  std::vector<cv::Mat> rgbWithPoints;
  std::vector<cv::Mat> raycastedPoses;

  std::vector<cv::Scalar> colours{CV_RGB(255, 0, 0), CV_RGB(0, 255, 0), CV_RGB(0, 0, 255)};

  for(size_t candidateIdx = 0; candidateIdx < candidates.size(); ++candidateIdx)
  {
    PoseCandidate &candidate = candidates[candidateIdx];

    ORUtils::SE3Pose pose;
    pose.SetInvM(candidate.cameraPose);

    model->get_visualisation_generator()->generate_voxel_visualisation(
        rendered, voxelScene, pose, view, liveVoxelRenderState, VisualisationGenerator::VT_SCENE_SEMANTICLAMBERTIAN);

    cv::Mat raycastedPose = cv::Mat(480, 640, CV_8UC4, rendered->GetData(MEMORYDEVICE_CPU)).clone();
    cv::cvtColor(raycastedPose, raycastedPose, CV_RGBA2BGR);

    // Draw Kabsch points in the images
    cv::Mat rgbKabsch = outRGB.clone();
    for(size_t i = 0; i < 3; ++i)
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
  for(size_t i = 0; i < 16; ++i)
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

  cv::imwrite("ransac_candidates.png", outCanvas);

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
      linearIdxDownsampled[i] = (view->depth->noDims.width / 4) * (ptCameraInt[i].y / 4) + ptCameraInt[i].x / 4;
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

        // Invert and transpose the covariance to print it in row-major
        Matrix3f posCovariance;
        m.positionInvCovariance.inv(posCovariance);
        posCovariance = posCovariance.t();

        for (int i = 0; i < 9; ++i) std::cout << posCovariance.m[i] << ' ';
        std::cout << '\n';
      }
      std::cout << '\n';
    }
#endif
  exit(0);
}
}
