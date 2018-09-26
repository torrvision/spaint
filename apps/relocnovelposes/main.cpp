/**
 * relocnovelposes: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Geometry>
#include <map>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "tvgutil/filesystem/PathFinder.h"
#include "tvgutil/timing/TimeUtil.h"

namespace fs = boost::filesystem;

//#################### CONSTANTS ####################

static const std::string trainFolderName = "train";
static const std::string validationFolderName = "validation";
static const std::string testFolderName = "test";

//#################### FUNCTIONS ####################

/**
 * \brief Finds the dataset sequences under a root folder.
 *
 *        The assumption is that each valid sequence folder will have both
 *        "train" and "test" subfolders.
 *
 * \param dataset_path The path to a dataset.
 *
 * \return A list of sequence names.
 */
std::vector<std::string> find_sequence_names(const fs::path &dataset_path)
{
  std::vector<std::string> sequences;

  // Iterate over every subfolder of the dataset.
  for(fs::directory_iterator it(dataset_path), end; it != end; ++it)
  {
    fs::path p = it->path();
    fs::path train_path = p / trainFolderName;
    fs::path test_path = p / testFolderName;

    // If the current folder has both a train and test subfolder, we store its name as a valid sequence.
    if(fs::is_directory(train_path) && fs::is_directory(test_path))
    {
      sequences.push_back(p.filename().string());
    }
  }

  // Sort sequence names because the directory iterator does not ensure ordering
  std::sort(sequences.begin(), sequences.end());

  return sequences;
}

fs::path generate_path(const fs::path& basePath, const std::string& mask, int index)
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

std::vector<Eigen::Matrix4f> read_sequence_trajectory(const fs::path& basePath, const std::string& fileMask)
{
  std::vector<Eigen::Matrix4f> res;

  while (true)
  {
    const fs::path posePath = generate_path(basePath, fileMask, static_cast<int>(res.size()));

    if (!fs::is_regular(posePath))
      break;

    res.push_back(read_pose_from_file(posePath));
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

bool pose_matches(const Eigen::Matrix4f& gtPose, const Eigen::Matrix4f& testPose, float translationMaxError, float angleMaxError)
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
  Eigen::Matrix4f relocPose;
  Eigen::Matrix4f icpPose;

  bool relocSucceeded;
  bool icpSucceeded;

  int trainIdx;
  int testIdx;
};

typedef std::vector<TestICPPair> BinnedPoses;

struct BinStats
{
  size_t totalPoses;
  size_t successfulReloc;
  size_t successfulICP;

  float relocPct;
  float icpPct;

  BinStats()
  : totalPoses(0),
    successfulReloc(0),
    successfulICP(0),
    relocPct(0.0f),
    icpPct(0.0f)
  {}
};

struct ErrorThreshold
{
  float translationMaxError;
  float angleMaxError;

  template <typename T>
  ErrorThreshold(float translationMaxError_, T angleMaxError_)
  : translationMaxError(translationMaxError_), angleMaxError(static_cast<float>(angleMaxError_))
  {}
};

std::vector<BinnedPoses> classify_poses(
    const std::vector<Eigen::Matrix4f> &trainPoses,
    const std::vector<Eigen::Matrix4f> &testPoses,
    const std::vector<Eigen::Matrix4f> &relocPoses,
    const std::vector<Eigen::Matrix4f> &icpPoses,
    const std::vector<ErrorThreshold> &thresholds)
{
  std::vector<BinnedPoses> res;

  res.resize(thresholds.size() + 1);

  // For each test pose:
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int testIndex = 0; testIndex < static_cast<int>(testPoses.size()); ++testIndex)
  {
    const Eigen::Matrix4f& testPose = testPoses[testIndex];
    const Eigen::Matrix4f& relocPose = relocPoses[testIndex];
    const Eigen::Matrix4f& icpPose = icpPoses[testIndex];

//    if (!pose_matches(testPose, icpPose, 0.05f, 5.f * M_PI / 180.f))
//      continue;

    size_t closestTrainingIdx = std::numeric_limits<size_t>::max();
//    float closestTrainPoseDist = std::numeric_limits<float>::max();
    Eigen::Matrix4f closestTrainPose;
    closestTrainPose.setConstant(std::numeric_limits<float>::quiet_NaN());

    // Determine a difficulty bin for the test.
    size_t chosenBin = thresholds.size();
    for (size_t trainIndex = 0, trainCount = trainPoses.size(); trainIndex < trainCount; ++trainIndex)
    {
      const Eigen::Matrix4f& trainPose = trainPoses[trainIndex];
      for (size_t binIndex = 0; binIndex < chosenBin; ++binIndex)
      {
        if (pose_matches(trainPose, testPose,
            thresholds[binIndex].translationMaxError,
            thresholds[binIndex].angleMaxError))
        {
          if(binIndex < chosenBin)
          {
            closestTrainPose = trainPose; // There could be a closer one, but it's gonna be in the same bin, that is what we care about.
            closestTrainingIdx = trainIndex;
            chosenBin = binIndex;
            break;
          }
        }
      }
    }

    TestICPPair currentPoses;
    currentPoses.trainPose = closestTrainPose;
    currentPoses.testPose = testPose;
    currentPoses.relocPose = relocPose;
    currentPoses.icpPose = icpPose;
    currentPoses.relocSucceeded = pose_matches(testPose, relocPose, 0.05f, static_cast<float>(5.f * M_PI / 180.f));
    currentPoses.icpSucceeded = pose_matches(testPose, icpPose, 0.05f, static_cast<float>(5.f * M_PI / 180.f));
    currentPoses.trainIdx = static_cast<int>(closestTrainingIdx);
    currentPoses.testIdx = testIndex;

    // Store the test/icp pair in the right bucket
#ifdef WITH_OPENMP
    #pragma omp critical
#endif
    res[chosenBin].push_back(currentPoses);
  }

  return res;
}

std::vector<BinStats> compute_bin_stats(const std::vector<BinnedPoses>& binnedPoses)
{
  std::vector<BinStats> binStats(binnedPoses.size());

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int binIdx = 0; binIdx < static_cast<int>(binnedPoses.size()); ++binIdx)
  {
    const BinnedPoses& binContents = binnedPoses[binIdx];

    BinStats stats;
    stats.totalPoses = binContents.size();

    for(BinnedPoses::const_iterator it = binContents.begin(), iend = binContents.end(); it != iend; ++it)
    {
      stats.successfulReloc += it->relocSucceeded;
      stats.successfulICP += it->icpSucceeded;
    }

    stats.relocPct = static_cast<float>(stats.successfulReloc) / static_cast<float>(stats.totalPoses);
    stats.icpPct = static_cast<float>(stats.successfulICP) / static_cast<float>(stats.totalPoses);

    binStats[binIdx] = stats;
  }

  return binStats;
}

template<typename T>
void printWidth(const T &item, int width, bool leftAlign = false)
{
  std::cout << (leftAlign ? std::left : std::right) << std::setw(width)
      << std::fixed << std::setprecision(2) << item;
}

int main(int argc, char *argv[])
{
  std::vector<ErrorThreshold> errorThresholds;

  errorThresholds.push_back(ErrorThreshold(0.05f, 5.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.10f, 10.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.15f, 15.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.20f, 20.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.25f, 25.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.30f, 30.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.35f, 35.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.40f, 40.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.45f, 45.f * M_PI / 180.f));
  errorThresholds.push_back(ErrorThreshold(0.50f, 50.f * M_PI / 180.f));
//  errorThresholds.push_back(ErrorThreshold { 0.55f, 55.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.60f, 60.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.65f, 65.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.70f, 70.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.75f, 75.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.80f, 80.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.85f, 85.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.90f, 90.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 0.95f, 95.f * M_PI / 180.f});
//  errorThresholds.push_back(ErrorThreshold { 1.00f, 100.f * M_PI / 180.f});

  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0]
        << " \"dataset folder\" \"reloc output folder\" \"tag\""
        << std::endl;
    return 1;
  }

  fs::path datasetFolder = argv[1];
  fs::path relocBaseFolder = argv[2];
  std::string relocTag = argv[3];

  // Find the valid sequences in the dataset folder.
  const std::vector<std::string> sequenceNames = find_sequence_names(datasetFolder);

  std::map<std::string, std::vector<BinStats> > results;

  // Evaluate each sequence.
  for(size_t sequenceIdx = 0; sequenceIdx < sequenceNames.size(); ++sequenceIdx)
  {
    const std::string &sequence = sequenceNames[sequenceIdx];

    // Compute the full paths.
    const fs::path trainFolder = datasetFolder / sequence / trainFolderName;
    const fs::path testFolder = datasetFolder / sequence / testFolderName;
    const fs::path relocFolder = relocBaseFolder / (relocTag + '_' + sequence);

    std::cerr << "Processing sequence " << sequence << " in: " << testFolder << "\t - " << relocFolder << std::endl;

    std::vector<Eigen::Matrix4f> trainTrajectory = read_sequence_trajectory(trainFolder, "frame-%06i.pose.txt");
    std::cout << "Train: " << trainTrajectory.size() << " - ";

    std::vector<Eigen::Matrix4f> testTrajectory = read_sequence_trajectory(testFolder, "frame-%06i.pose.txt");
    std::cout << "Test: " << testTrajectory.size() << " - ";

    std::vector<Eigen::Matrix4f> relocTrajectory = read_sequence_trajectory(relocFolder, "pose-%06i.reloc.txt");
    std::cout << "Reloc: " << relocTrajectory.size() << " - ";

    std::vector<Eigen::Matrix4f> icpTrajectory = read_sequence_trajectory(relocFolder, "pose-%06i.icp.txt");
    std::cout << "ICP: " << icpTrajectory.size() << "\n";

    // Classify poses
    std::vector<BinnedPoses> binnedPoses = classify_poses(trainTrajectory, testTrajectory, relocTrajectory, icpTrajectory, errorThresholds);

    // Compute stats for each bin.
    std::vector<BinStats> binStats = compute_bin_stats(binnedPoses);

    results[sequence] = binStats;
  }

  // Print table.
  printWidth("Sequence", 15, true);
  printWidth("Bin T.", 8);
  printWidth("Bin A.", 8);
  printWidth("Pose Ct", 8);
  printWidth("Reloc %", 8);
  printWidth("ICP %", 8);
  printWidth("Success R", 10);
  printWidth("Failure R", 10);
  printWidth("Success I", 10);
  printWidth("Failure I", 10);
  std::cerr << "\n";

  // For each sequence:
  for(std::map<std::string,std::vector<BinStats> >::const_iterator it = results.begin(), iend = results.end(); it != iend; ++it)
  {
    const std::vector<BinStats>& bins = it->second;

    // For each bin:
    for(size_t binIdx = 0; binIdx < bins.size(); ++binIdx)
    {
      const BinStats& binStats = bins[binIdx];
      const float distThresh = binIdx < errorThresholds.size() ? errorThresholds[binIdx].translationMaxError : std::numeric_limits<float>::infinity();
      const float angleThresh = binIdx < errorThresholds.size() ? errorThresholds[binIdx].angleMaxError * 180.0f / static_cast<float>(M_PI) : std::numeric_limits<float>::infinity();

      if(binIdx == 0)
      {
        printWidth(it->first, 15, true);
      }
      else
      {
        printWidth("", 15);
      }

      printWidth(distThresh, 8);
      printWidth(angleThresh, 8);
      printWidth(binStats.totalPoses, 8);
      printWidth(binStats.relocPct, 8);
      printWidth(binStats.icpPct, 8);
      printWidth(binStats.successfulReloc, 10);
      printWidth(binStats.totalPoses - binStats.successfulReloc, 10);
      printWidth(binStats.successfulICP, 10);
      printWidth(binStats.totalPoses - binStats.successfulICP, 10);
      std::cerr << "\n";
    }
  }

//  std::cout << "Stats:\n";
//  std::cout << std::fixed << std::setprecision(3);
//  for(size_t i = 0; i < binStats.size(); ++i)
//  {
//    float distThresh = i < errorThresholds.size() ? errorThresholds[i].translationMaxError : std::numeric_limits<float>::infinity();
//    float angleThresh = i < errorThresholds.size() ? errorThresholds[i].angleMaxError * 180.0f / M_PI : std::numeric_limits<float>::infinity();
//    std::cout << i << ": " << distThresh << "/" << angleThresh << " - " << binStats[i].relocPct << " - " << binStats[i].icpPct << "\n";
//  }

  return 0;
}
