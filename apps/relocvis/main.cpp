/**
 * relocvis: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Geometry>
#include <map>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include "tvgutil/timing/TimeUtil.h"

namespace fs = boost::filesystem;
using namespace cv::viz;

//#################### FUNCTIONS ####################

fs::path generate_path(const fs::path basePath, const std::string &mask,
    int index)
{
  char buf[2048];
  sprintf(buf, mask.c_str(), index);

  return basePath / buf;
}

Eigen::Matrix4f read_pose_from_file(const fs::path &fileName)
{
  if (!fs::is_regular(fileName))
    throw std::runtime_error("File not found: " + fileName.string());

  std::ifstream in(fileName.c_str());

  Eigen::Matrix4f res;

  in >> res(0, 0) >> res(0, 1) >> res(0, 2) >> res(0, 3);
  in >> res(1, 0) >> res(1, 1) >> res(1, 2) >> res(1, 3);
  in >> res(2, 0) >> res(2, 1) >> res(2, 2) >> res(2, 3);
  in >> res(3, 0) >> res(3, 1) >> res(3, 2) >> res(3, 3);

  return res;
}

std::vector<Eigen::Matrix4f> read_sequence_trajectory(const fs::path &basePath,
    const std::string &fileMask)
{
  std::vector<Eigen::Matrix4f> res;

  while (true)
  {
    const fs::path posePath = generate_path(basePath, fileMask, res.size());

    if (!fs::is_regular(posePath))
      break;

    res.push_back(read_pose_from_file(posePath));
  }

  return res;
}

std::vector<cv::Affine3f> eigen_to_cv(
    const std::vector<Eigen::Matrix4f> &trajectory)
{
  std::vector<cv::Affine3f> res;
  for (auto eigM : trajectory)
  {
    cv::Affine3f cvM;
    for (int i = 0; i < 16; ++i)
      cvM.matrix.val[i] = eigM.data()[i];

    // Col Major to row major
    cvM.matrix = cvM.matrix.t();

    res.push_back(cvM);
  }

  return res;
}

std::vector<cv::Affine3f> subsample_trajectory(
    const std::vector<cv::Affine3f> &trajectory, int factor)
{
  std::vector<cv::Affine3f> res;

  for (size_t i = 0; i < trajectory.size(); i += factor)
  {
    res.push_back(trajectory[i]);
  }

  return res;
}

std::vector<std::vector<cv::Affine3f>> split_trajectory(
    const std::vector<cv::Affine3f> &trajectory, int every)
{
  std::vector<std::vector<cv::Affine3f>> res;

  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    if (i % every == 0)
      res.push_back(std::vector<cv::Affine3f>());

    res.back().push_back(trajectory[i]);
  }

  return res;
}

float angular_separation(const Eigen::Matrix3f& r1, const Eigen::Matrix3f& r2)
{
  // First calculate the rotation matrix which maps r1 to r2.
  Eigen::Matrix3f dr = r2 * r1.transpose();

  Eigen::AngleAxisf aa(dr);
  return aa.angle();
}

bool pose_matches(const Eigen::Matrix4f &gtPose,
    const Eigen::Matrix4f &testPose, float translationMaxError,
    float angleMaxError)
{
  const Eigen::Matrix3f gtR = gtPose.block<3, 3>(0, 0);
  const Eigen::Matrix3f testR = testPose.block<3, 3>(0, 0);
  const Eigen::Vector3f gtT = gtPose.block<3, 1>(0, 3);
  const Eigen::Vector3f testT = testPose.block<3, 1>(0, 3);

  const float translationError = (gtT - testT).norm();
  const float angleError = angular_separation(gtR, testR);
//  const float angleError = angular_separation_approx(gtR, testR);

  return translationError <= translationMaxError && angleError <= angleMaxError;
}

struct TestICPPair
{
  Eigen::Matrix4f trainPose;
  Eigen::Matrix4f testPose;
  Eigen::Matrix4f icpPose;

  size_t trainIdx;
  size_t testIdx;
};

struct ErrorThreshold
{
  float translationMaxError;
  float angleMaxError;
  Color color;
};

std::vector<std::vector<TestICPPair>> classify_poses(
    const std::vector<Eigen::Matrix4f> &trainPoses,
    const std::vector<Eigen::Matrix4f> &testPoses,
    const std::vector<Eigen::Matrix4f> &icpPoses,
    const std::vector<ErrorThreshold> &thresholds)
{
  std::vector<std::vector<TestICPPair>> res;

  res.resize(thresholds.size() + 1);

  // For each test pose:
  for (size_t testIndex = 0, testCount = testPoses.size();
      testIndex < testCount; ++testIndex)
  {
    const Eigen::Matrix4f& testPose = testPoses[testIndex];
    const Eigen::Matrix4f& icpPose = icpPoses[testIndex];

    if (!pose_matches(testPose, icpPose, 0.05f, 5.f * M_PI / 180.f))
      continue;

    size_t closestTrainingIdx = std::numeric_limits<size_t>::max();
    float closestTrainPoseDist = std::numeric_limits<float>::max();
    Eigen::Matrix4f closestTrainPose;
    closestTrainPose.setConstant(std::numeric_limits<float>::quiet_NaN());

    // Determine a difficulty bin for the test.
    size_t chosenBin = thresholds.size();
    for (size_t trainIndex = 0, trainCount = trainPoses.size();
        trainIndex < trainCount; ++trainIndex)
    {
      const Eigen::Matrix4f& trainPose = trainPoses[trainIndex];
      for (size_t binIndex = 0; binIndex < chosenBin; ++binIndex)
      {
        if (pose_matches(trainPose, testPose,
            thresholds[binIndex].translationMaxError,
            thresholds[binIndex].angleMaxError))
        {
//          if ((trainPose.block<3, 1>(0, 3) - testPose.block<3, 1>(0, 3)).norm()
//              < closestTrainPoseDist)
          {
            closestTrainPoseDist = (trainPose.block<3, 1>(0, 3)
                - testPose.block<3, 1>(0, 3)).norm();
            closestTrainPose = trainPose;
            closestTrainingIdx = trainIndex;
            chosenBin = binIndex;
          }
        }
      }
    }

    // Store the test/icp pair in the right bucket
    res[chosenBin].push_back(TestICPPair
    { closestTrainPose, testPose, icpPose, closestTrainingIdx, testIndex });
  }

  return res;
}

std::vector<TestICPPair> prune_near_poses(const std::vector<TestICPPair> &poses)
{
  std::vector<TestICPPair> result;
  for (const TestICPPair &pose : poses)
  {
    Eigen::Vector3f candidateT = pose.icpPose.block<3, 1>(0, 3);

    // If there are no similar poses in the results vector insert the current candidate
    bool insert = true;
    for (size_t i = 0; i < result.size() && insert; ++i)
    {
      auto &resultPose = result[i];
      Eigen::Vector3f resultT = resultPose.icpPose.block<3, 1>(0, 3);
      Eigen::Vector3f diff = candidateT - resultT;

      if (diff.norm() < 0.5f)
      {
        insert = false;
      }
    }

    if (insert)
      result.push_back(pose);
  }

  return result;
}

template<typename T>
void printWidth(const T &item, int width, bool leftAlign = false)
{
  std::cout << (leftAlign ? std::left : std::right) << std::setw(width)
      << std::fixed << std::setprecision(2) << item;
}

struct Cookie
{
  bool quit;
  bool refresh;
  bool screenshot;
};

void key_cb(const KeyboardEvent& event, void* cookie_v)
{
  Cookie *cookie = (Cookie*) cookie_v;
  if (event.action == KeyboardEvent::KEY_DOWN)
  {
    if (event.code == 27)
      cookie->quit = true;
    if (event.code == ' ')
      cookie->refresh = true;
    if (event.code == 'm')
      cookie->screenshot = true;
  }
}

int main(int argc, char *argv[])
{
  static const int TRAJ_SUB_FACTOR = 100;
  static const int SUBTRAJ_LENGTH = 1000;
  static const cv::Matx33d intrinsics(585, 0, 320, 0, 585, 240, 0, 0, 1);

  std::vector<ErrorThreshold> errorThresholds;

//  errorThresholds.push_back(ErrorThreshold
//  { 0.05f, 5.f * M_PI / 180.f, Color(0x00, 0xFF, 0x00) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.10f, 10.f * M_PI / 180.f, Color(0x00, 0xF4, 0x2A) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.20f, 20.f * M_PI / 180.f, Color(0x00, 0xAA, 0x55) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.30f, 30.f * M_PI / 180.f, Color(0x00, 0x7F, 0x7F) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.40f, 40.f * M_PI / 180.f, Color(0x00, 0x55, 0xAA) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.50f, 50.f * M_PI / 180.f, Color(0x00, 0x2A, 0xD4) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.7f, 70.f * M_PI / 180.f, Color(0x00, 0x00, 0xFF) });

//  errorThresholds.push_back(ErrorThreshold
//  { 0.10f, 10.f * M_PI / 180.f, Color(0x00, 0xFF, 0x00) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.30f, 30.f * M_PI / 180.f, Color(0x00, 0xA5, 0xFF) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.60f, 60.f * M_PI / 180.f, Color(0x00, 0x00, 0xFF) });

  errorThresholds.push_back(ErrorThreshold
  { 0.10f, 10.f * M_PI / 180.f, Color(0x00, 0xFF, 0x00) });
  errorThresholds.push_back(ErrorThreshold
  { 0.35f, 35.f * M_PI / 180.f, Color(0x00, 0xA5, 0xFF) });
  errorThresholds.push_back(ErrorThreshold
  { 0.7f, 70.f * M_PI / 180.f, Color(0x00, 0x00, 0xFF) });

//  errorThresholds.push_back(ErrorThreshold
//  { 0.05f, 5.f * M_PI / 180.f, Color(0x00, 0xFF, 0x00) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.15f, 15.f * M_PI / 180.f, Color(0x00, 0xBF, 0x3F) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.30f, 30.f * M_PI / 180.f, Color(0x00, 0x7F, 0x7F) });
//  errorThresholds.push_back(ErrorThreshold
//  { 0.45f, 45.f * M_PI / 180.f, Color(0x00, 0x3F, 0xBF) });

  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0]
        << " \"train folder\" \"test folder\" \"reloc output folder\" \"mesh\""
        << std::endl;
    return 1;
  }

  fs::path trainFolder = argv[1];
  fs::path testFolder = argv[2];
  fs::path relocFolder = argv[3];
  fs::path meshPath = argv[4];

  Mesh seqMesh = Mesh::load(meshPath.string(), Mesh::LOAD_OBJ);
  WMesh meshWidget(seqMesh);

  std::cout << "Loaded mesh from: " << meshPath << "\n\tVertices: "
      << seqMesh.cloud.cols << "\n\tTriangles: " << seqMesh.polygons.cols
      << '\n';

  std::vector<Eigen::Matrix4f> trainTrajectory = read_sequence_trajectory(
      trainFolder, "posem%06i.txt");
  auto trainTrajectorySplit = split_trajectory(eigen_to_cv(trainTrajectory),
      SUBTRAJ_LENGTH);
//  std::vector<cv::Affine3f> trainTrajectorySub = subsample_trajectory(
//      trainTrajectory, 10);
  std::cout << "Training trajectory has " << trainTrajectory.size()
      << " poses. Subtrajectories: " << trainTrajectorySplit.size() << "\n";

  std::vector<Eigen::Matrix4f> testTrajectory = read_sequence_trajectory(
      testFolder, "posem%06i.txt");
//  auto testTrajectorySub = subsample_trajectory(
//      testTrajectory, TRAJ_SUB_FACTOR);
//  std::cout << "Testing trajectory has " << testTrajectory.size()
//      << " poses.\n";

  std::vector<Eigen::Matrix4f> icpTrajectory = read_sequence_trajectory(
      relocFolder, "pose-%06i.icp.txt");
  auto icpTrajectorySplit = split_trajectory(eigen_to_cv(icpTrajectory),
      SUBTRAJ_LENGTH);
//  std::vector<cv::Affine3f> icpTrajectorySub = subsample_trajectory(
//      icpTrajectory, TRAJ_SUB_FACTOR);
  std::cout << "ICP trajectory has " << icpTrajectory.size()
      << " poses. Subtrajectories: " << icpTrajectorySplit.size() << "\n";

// Visualizer
  Viz3d visualizer("relocviz");
  Viz3d visualizerTrajectory("relocviz-trajectory");

// show training trajectory
  {
    Color col = Color::yellow();
    for (size_t i = 0; i < trainTrajectorySplit.size(); ++i)
    {
      std::string tName = "trainTrajectory_"
          + boost::lexical_cast<std::string>(i);
      WTrajectory traj(trainTrajectorySplit[i], WTrajectory::PATH, 0.1, col);
      visualizerTrajectory.showWidget(tName, traj);
      visualizerTrajectory.setRenderingProperty(tName, LINE_WIDTH, 2);

      visualizer.showWidget(tName, traj);
      visualizer.setRenderingProperty(tName, LINE_WIDTH, 2);

//      auto subsampledTraj = subsample_trajectory(trainTrajectorySplit[i], 60);
//      WTrajectory trajF(subsampledTraj, WTrajectory::FRAMES, 0.15, col);
//      visualizer.showWidget(tName + "F", trajF);
//      visualizer.setRenderingProperty(tName + "F", LINE_WIDTH, 2);
    }
  }

//// show ICP trajectory
//  {
//    Color col = Color::apricot();
//    for (size_t i = 0; i < icpTrajectorySplit.size(); ++i)
//    {
//      std::string tName = "icpTrajectory_"
//          + boost::lexical_cast<std::string>(i);
//      auto subsampled = subsample_trajectory(icpTrajectorySplit[i], 50);
//      WTrajectoryFrustums traj(subsampled, intrinsics, 0.25, col);
//      visualizer.showWidget(tName, traj);
//      visualizer.setRenderingProperty(tName, LINE_WIDTH, 3);
//    }
//  }

// Classify poses
  auto classifiedPoses = classify_poses(trainTrajectory, testTrajectory,
      icpTrajectory, errorThresholds);

  // Prune nearby poses
  for (size_t binIdx = 0; binIdx < classifiedPoses.size(); ++binIdx)
  {
    auto &posePairs = classifiedPoses[binIdx];
    std::cout << "The bin " << binIdx << " has " << posePairs.size()
        << " poses.\n";

    posePairs = prune_near_poses(posePairs);
    std::cout << "After pruning closest poses has " << posePairs.size()
        << " poses.\n";
  }

//  // Shuffle them for visualization
//  for (size_t binIdx = 0; binIdx < classifiedPoses.size(); ++binIdx)
//  {
//    std::random_shuffle(classifiedPoses[binIdx].begin(),
//        classifiedPoses[binIdx].end());
//  }

// Show some pose examples
  std::vector<std::string> trainExamplePaths;
  std::vector<std::string> testExamplePaths;

  for (int binIdx = classifiedPoses.size() - 2;
      binIdx >= 0 && trainExamplePaths.size() < 8; --binIdx)
  {
    const auto &posePairs = classifiedPoses[binIdx];
    for (size_t pairIdx = 0;
        pairIdx < posePairs.size() && trainExamplePaths.size() < 8; ++pairIdx)
    {
      const auto &pair = posePairs[pairIdx];
      trainExamplePaths.push_back(
          generate_path(trainFolder, "rgbm%06i.ppm", pair.trainIdx).string());
      testExamplePaths.push_back(
          generate_path(testFolder, "rgbm%06i.ppm", pair.testIdx).string());
    }
  }

  static const int IMG_WIDTH = 640;
  static const int IMG_HEIGHT = 480;
  static const int IMG_STEP = 5;
  cv::Mat samplePoseImg = cv::Mat::zeros(IMG_HEIGHT * 2 + IMG_STEP,
      IMG_WIDTH * trainExamplePaths.size()
          + IMG_STEP * (trainExamplePaths.size() - 1),
      CV_8UC3);

  for (size_t sampleIdx = 0; sampleIdx < trainExamplePaths.size(); ++sampleIdx)
  {
    std::string trainPath = trainExamplePaths[sampleIdx];
    std::string testPath = testExamplePaths[sampleIdx];
    std::cout << "Loading training image from: " << trainPath
        << "\nLoading test image from: " << testPath << '\n';

    cv::Mat trainImg = cv::imread(trainPath);
    cv::Mat testImg = cv::imread(testPath);

    cv::Rect trainRect(sampleIdx * (IMG_WIDTH + IMG_STEP), 0, IMG_WIDTH,
        IMG_HEIGHT);
    cv::Rect testRect(sampleIdx * (IMG_WIDTH + IMG_STEP), IMG_HEIGHT + IMG_STEP,
        IMG_WIDTH, IMG_HEIGHT);

    trainImg.copyTo(samplePoseImg(trainRect));
    testImg.copyTo(samplePoseImg(testRect));
  }

  cv::namedWindow("Sample images", cv::WINDOW_KEEPRATIO);
  cv::imshow("Sample images", samplePoseImg);

  int key = 0;
  while (key != 27)
  {
    key = cv::waitKey();
  }

  cv::destroyAllWindows();

//  cv::Mat trainImg = cv::imread(trainExamplePaths[0]);
//  cv::Mat testImg = cv::imread(testExamplePaths[0]);
//
//  cv::imshow("Train", trainImg);
//  cv::imshow("Test", testImg);
//  cv::waitKey();

  for (size_t binIdx = 0; binIdx < classifiedPoses.size(); ++binIdx)
  {
    auto &posePairs = classifiedPoses[binIdx];
    // Show at most N poses for each bin
    posePairs.resize(std::min<size_t>(posePairs.size(), 4));
//    posePairs.resize(std::min<size_t>(posePairs.size() / 10, 10));

// Extract the poses from the pairs
    std::vector<Eigen::Matrix4f> trainPoses, icpPoses;
    for (const auto &x : posePairs)
    {
      trainPoses.push_back(x.trainPose);
      icpPoses.push_back(x.icpPose);
    }

    auto trainPosesCv = eigen_to_cv(trainPoses);
    auto icpPosesCv = eigen_to_cv(icpPoses);

    Color posesColor =
        binIdx < errorThresholds.size() ?
            errorThresholds[binIdx].color : Color::red();

    // Draw the training camera frustums
    {
      std::string tName = "trainPose_"
          + boost::lexical_cast<std::string>(binIdx);
//    auto subsampled = subsample_trajectory(posesCv, 50);
      WTrajectoryFrustums traj(trainPosesCv, intrinsics, 0.15,
          Color::bluberry());
      visualizer.showWidget(tName, traj);
      visualizer.setRenderingProperty(tName, LINE_WIDTH, 3);

      visualizerTrajectory.showWidget(tName, traj);
      visualizerTrajectory.setRenderingProperty(tName, LINE_WIDTH, 3);
    }

    // Draw the lines connecting them
    for (size_t i = 0; i < trainPosesCv.size(); ++i)
    {
      const cv::Affine3f &trainPose = trainPosesCv[i];
      const cv::Affine3f &icpPose = icpPosesCv[i];

      WLine wLine(cv::Point3f(trainPose.translation()),
          cv::Point3f(icpPose.translation()), Color::magenta());
      std::string wName = "trainTest_"
          + boost::lexical_cast<std::string>(binIdx) + '_'
          + boost::lexical_cast<std::string>(i);
      visualizer.showWidget(wName, wLine);
      visualizer.setRenderingProperty(wName, LINE_WIDTH, 3);
    }

    {
      std::string tName = "icpTrajectory_"
          + boost::lexical_cast<std::string>(binIdx);
//    auto subsampled = subsample_trajectory(posesCv, 50);
      WTrajectoryFrustums traj(icpPosesCv, intrinsics, 0.15, posesColor);
      visualizer.showWidget(tName, traj);
      visualizer.setRenderingProperty(tName, LINE_WIDTH, 3);
    }
  }

//  WTrajectory trainTrajectoryW(trainTrajectory, WTrajectory::PATH, 1.0,
//      Color::green());
//  WTrajectoryFrustums trainTrajectoryF(trainTrajectorySub, intrinsics, 0.05,
//      Color::green());

//  WTrajectory testTrajectoryW(testTrajectory, WTrajectory::PATH, 1.0,
//      Color::blue());
//  WTrajectoryFrustums testTrajectoryF(testTrajectorySub, intrinsics, 0.25,
//      Color::blue());
//
//  WTrajectory icpTrajectoryW(icpTrajectory, WTrajectory::PATH, 1.0,
//      Color::red());
//  WTrajectoryFrustums icpTrajectoryF(icpTrajectorySub, intrinsics, 0.25,
//      Color::red());

  visualizer.showWidget("meshWidget", meshWidget);
  visualizerTrajectory.showWidget("meshWidget", meshWidget);
//  visualizer.setRenderingProperty("meshWidget", SHADING, SHADING_PHONG);

//  visualizer.showWidget("trainTrajectoryW", trainTrajectoryW);
//  visualizer.showWidget("trainTrajectoryF", trainTrajectoryF);

//  visualizer.showWidget("testTrajectoryW", testTrajectoryW);
//  visualizer.showWidget("testTrajectoryF", testTrajectoryF);
//
//  visualizer.showWidget("icpTrajectoryW", icpTrajectoryW);
//  visualizer.showWidget("icpTrajectoryF", icpTrajectoryF);

  visualizer.resetCamera();

  Cookie cookie;
  cookie.quit = false;
  cookie.refresh = false;
  cookie.screenshot = false;

  visualizer.registerKeyboardCallback(key_cb, &cookie);
//  visualizerTrajectory.registerKeyboardCallback(key_cb, &cookie);

  while (!cookie.quit)
  {
    visualizer.spinOnce();

    if (cookie.refresh)
    {
      auto camera = visualizer.getViewerPose();
      visualizerTrajectory.setViewerPose(camera);
      visualizerTrajectory.spinOnce();
      cookie.refresh = false;
    }

    if (cookie.screenshot)
    {
//      visualizerTrajectory.spinOnce();

      std::string timestamp = tvgutil::TimeUtil::get_iso_timestamp();

      visualizer.saveScreenshot(timestamp + "-novelPoses.png");
      visualizerTrajectory.saveScreenshot(timestamp + "-trajectory.png");

      cookie.screenshot = false;
    }
  }

  return 0;
}
