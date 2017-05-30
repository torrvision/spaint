/**
 * relocperf: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <map>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs = boost::filesystem;

static const std::string trainFolderName = "train";
static const std::string validationFolderName = "validation";
static const std::string testFolderName = "test";

//#################### FUNCTIONS ####################

std::vector<std::string> find_sequence_names(const fs::path &dataset_path)
{
  std::vector<std::string> sequences;

  for(fs::directory_iterator it(dataset_path), end; it != end; ++it)
  {
    fs::path p = it->path();
    fs::path train_path = p / trainFolderName;
    fs::path test_path = p / testFolderName;

    if(fs::is_directory(train_path) && fs::is_directory(test_path))
    {
      sequences.push_back(p.filename().string());
    }
  }

  // Sort sequence names because the directory iterator does not ensure ordering
  std::sort(sequences.begin(), sequences.end());

  return sequences;
}

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

float angular_separation(const Eigen::Vector3f& t1, const Eigen::Vector3f& t2)
{
  float divisor = t1.norm() * t1.norm();
  if (divisor > 0.0f)
  {
    float cosineTheta = t1.dot(t2) / divisor;
    if (cosineTheta > 1.0f)
      cosineTheta = 1.0f;
    return acos(cosineTheta);
  }
  else
    return 0.0f;
}

float angular_separation_approx(const Eigen::Matrix3f& r1,
    const Eigen::Matrix3f& r2)
{
  // Rotate a vector with each rotation matrix and return the angular difference.
  Eigen::Vector3f v(1, 1, 1);
  Eigen::Vector3f v1 = r1 * v;
  Eigen::Vector3f v2 = r2 * v;
  return angular_separation(v1, v2);
}

float angular_separation(const Eigen::Matrix3f& r1, const Eigen::Matrix3f& r2)
{
  // First calculate the rotation matrix which maps r1 to r2.
  Eigen::Matrix3f dr = r2 * r1.transpose();

  Eigen::AngleAxisf aa(dr);
  return aa.angle();
}

bool pose_matches(const Eigen::Matrix4f &gtPose,
    const Eigen::Matrix4f &testPose)
{
  static const float translationMaxError = 0.05f;
  static const float angleMaxError = 5.f * M_PI / 180.f;

  const Eigen::Matrix3f gtR = gtPose.block<3, 3>(0, 0);
  const Eigen::Matrix3f testR = testPose.block<3, 3>(0, 0);
  const Eigen::Vector3f gtT = gtPose.block<3, 1>(0, 3);
  const Eigen::Vector3f testT = testPose.block<3, 1>(0, 3);

  const float translationError = (gtT - testT).norm();
  const float angleError = angular_separation(gtR, testR);
//  const float angleError = angular_separation_approx(gtR, testR);

  return translationError <= translationMaxError && angleError <= angleMaxError;
}

bool pose_file_matches(const Eigen::Matrix4f &gtPose, const fs::path &poseFile)
{
  if(!fs::is_regular(poseFile))
    return false;

  const Eigen::Matrix4f otherPose = read_pose_from_file(poseFile);
  return pose_matches(gtPose, otherPose);
}

struct SequenceResults
{
  int poseCount
  { 0 };
  int validPosesAfterReloc
  { 0 };
  int validPosesAfterICP
  { 0 };
  int validFinalPoses
  { 0 };

  std::vector<bool> relocalizationResults;
  std::vector<bool> icpResults;
  std::vector<bool> finalResults;
};

SequenceResults evaluate_sequence(const fs::path &gtFolder,
    const fs::path &relocFolder)
{
  SequenceResults res;

  while (true)
  {
    const fs::path gtPath = generate_path(gtFolder, "frame-%06i.pose.txt", res.poseCount);
    const fs::path relocPath = generate_path(relocFolder, "pose-%06i.reloc.txt",
        res.poseCount);
    const fs::path icpPath = generate_path(relocFolder, "pose-%06i.icp.txt",
        res.poseCount);
    const fs::path finalPath = generate_path(relocFolder, "pose-%06i.final.txt",
        res.poseCount);

    if (!fs::is_regular(gtPath))
      break;

//    std::cout << "Reading GT from: " << gtPath << '\n';
//    std::cout << "Reading relocalised pose from: " << relocPath << '\n';
//    std::cout << "Reading refined pose from: " << icpPath << '\n';

    const Eigen::Matrix4f gtPose = read_pose_from_file(gtPath);
    const Eigen::Matrix4f relocPose = read_pose_from_file(relocPath);
    const Eigen::Matrix4f icpPose = read_pose_from_file(icpPath);
//    const Eigen::Matrix4f finalPose = read_pose_from_file(finalPath);

    bool validReloc = pose_matches(gtPose, relocPose);
    bool validICP = pose_matches(gtPose, icpPose);
//    bool validFinal = pose_matches(gtPose, finalPose);

//    bool validReloc = pose_file_matches(gtPose, relocPath);
//    bool validICP = pose_file_matches(gtPose, icpPath);
//    bool validFinal = pose_file_matches(gtPose, finalPath);

//    std::cout << res.poseCount << "-> Reloc: " << std::boolalpha << validReloc
//        << " - ICP: " << validICP << std::noboolalpha << '\n';

    ++res.poseCount;
    res.validPosesAfterReloc += validReloc;
    res.validPosesAfterICP += validICP;
//    res.validFinalPoses += validFinal;

    res.relocalizationResults.push_back(validReloc);
    res.icpResults.push_back(validICP);
//    res.finalResults.push_back(validFinal);
  }

  return res;
}

template<typename T>
void printWidth(const T &item, int width, bool leftAlign = false)
{
  std::cout << (leftAlign ? std::left : std::right) << std::setw(width)
      << std::fixed << std::setprecision(2) << item;
}

int main(int argc, char *argv[])
{
  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0]
        << " \"GT base folder\" \"reloc base output folder\" \"reloc tag\" [online results filename]"
        << std::endl;
    return 1;
  }

  const fs::path gtFolder = argv[1];
  const fs::path relocBaseFolder = argv[2];
  const std::string relocTag = argv[3];
  const std::vector<std::string> sequenceNames = find_sequence_names(gtFolder);

  std::map<std::string, SequenceResults> results;

  for (auto sequence : sequenceNames)
  {
    const fs::path gtPath = gtFolder / sequence / testFolderName;
    const fs::path relocFolder = relocBaseFolder / (relocTag + '_' + sequence);

    std::cerr << "Processing sequence " << sequence << " in: " << gtPath
        << "\t - " << relocFolder << std::endl;
    try
    {
      results[sequence] = evaluate_sequence(gtPath, relocFolder);
    } catch (std::runtime_error&)
    {
      std::cerr << "\tSequence has not been evaluated.\n";
    }
  }

  // Print table
  printWidth("Sequence", 15, true);
  printWidth("Poses", 8);
  printWidth("Reloc", 8);
  printWidth("ICP", 8);
  printWidth("Final", 8);
  std::cout << '\n';

  for (const auto &sequence : sequenceNames)
  {
    const auto &seqResult = results[sequence];

    float relocPct = static_cast<float>(seqResult.validPosesAfterReloc)
        / static_cast<float>(seqResult.poseCount) * 100.f;
    float icpPct = static_cast<float>(seqResult.validPosesAfterICP)
        / static_cast<float>(seqResult.poseCount) * 100.f;
    float finalPct = static_cast<float>(seqResult.validFinalPoses)
        / static_cast<float>(seqResult.poseCount) * 100.f;

    printWidth(sequence, 15, true);
    printWidth(seqResult.poseCount, 8);
    printWidth(relocPct, 8);
    printWidth(icpPct, 8);
    printWidth(finalPct, 8);
    std::cout << '\n';
  }

  // Compute average performance
  float relocSum = 0.f;
  float icpSum = 0.f;
  float finalSum = 0.f;

  float relocRawSum = 0.f;
  float icpRawSum = 0.f;
  float finalRawSum = 0.f;
  int poseCount = 0;

  for (const auto &sequence : sequenceNames)
  {
    const auto &seqResult = results[sequence];

    // Non-weighted average, we need percentages
    const float relocPct = static_cast<float>(seqResult.validPosesAfterReloc)
        / static_cast<float>(seqResult.poseCount);
    const float icpPct = static_cast<float>(seqResult.validPosesAfterICP)
        / static_cast<float>(seqResult.poseCount);
    const float finalPct = static_cast<float>(seqResult.validFinalPoses)
        / static_cast<float>(seqResult.poseCount);

    relocSum += relocPct;
    icpSum += icpPct;
    finalSum += finalPct;

    relocRawSum += static_cast<float>(seqResult.validPosesAfterReloc);
    icpRawSum += static_cast<float>(seqResult.validPosesAfterICP);
    finalRawSum += static_cast<float>(seqResult.validFinalPoses);
    poseCount += seqResult.poseCount;
  }

  const float relocAvg = relocSum / sequenceNames.size() * 100.f;
  const float icpAvg = icpSum / sequenceNames.size() * 100.f;
  const float finalAvg = finalSum / sequenceNames.size() * 100.f;

  const float relocWeightedAvg = relocRawSum / poseCount * 100.f;
  const float icpWeightedAvg = icpRawSum / poseCount * 100.f;
  const float finalWeightedAvg = finalRawSum / poseCount * 100.f;

  // Print averages
  std::cout << '\n';
  printWidth("Average", 15, true);
  printWidth(sequenceNames.size(), 8);
  printWidth(relocAvg, 8);
  printWidth(icpAvg, 8);
  printWidth(finalAvg, 8);
  std::cout << '\n';
  printWidth("Average (W)", 15, true);
  printWidth(poseCount, 8);
  printWidth(relocWeightedAvg, 8);
  printWidth(icpWeightedAvg, 8);
  printWidth(finalWeightedAvg, 8);
  std::cout << '\n';

  // Save results of online training-relocalization
  if (argc > 4)
  {
    std::string onlineResultsFilenameStem = argv[4];

    // Process every sequence
    for (auto sequence : sequenceNames)
    {
      auto seqResult = results[sequence];

      std::string outFilename = onlineResultsFilenameStem + '_' + sequence
          + ".csv";
      std::ofstream out(outFilename);

      // Print header
      out
          << "FrameIdx; FramePct; Reloc Success; Reloc Sum; Reloc Pct; ICP Success; ICP Sum; ICP Pct\n";

      int relocSum = 0;
      int icpSum = 0;

      for (int poseIdx = 0; poseIdx < seqResult.poseCount; ++poseIdx)
      {
        bool relocSuccess = seqResult.relocalizationResults[poseIdx];
        bool icpSuccess = seqResult.icpResults[poseIdx];

        relocSum += relocSuccess;
        icpSum += icpSuccess;

        float framePct = static_cast<float>(poseIdx) / seqResult.poseCount;
        float relocPct = static_cast<float>(relocSum) / poseIdx;
        float icpPct = static_cast<float>(icpSum) / poseIdx;

        out << poseIdx << "; " << framePct << "; " << relocSuccess << "; "
            << relocSum << "; " << relocPct << "; " << icpSuccess << "; "
            << icpSum << "; " << icpPct << '\n';
      }
    }
  }

#if 0
  std::cout << "\n\n";

  float colorMin = 40;
  float colorMax = 95;
  std::string relocColor = "ForestGreen";
  std::string icpColor = "CornflowerBlue";

  // Print Latex table row
  std::cout << "\\cellcolor{white} & \\cellcolor{" << relocColor << "} Reloc ";
  for (auto sequence : sequenceNames)
  {
    auto seqResult = results[sequence];

    float relocPct = static_cast<float>(seqResult.validPosesAfterReloc)
    / static_cast<float>(seqResult.poseCount) * 100.f;
    float relocColorPct = (relocPct / 100.f) * (colorMax - colorMin) + colorMin;

    std::cout << "& \\cellcolor{" << relocColor << "!" << relocColorPct << "} "
    << std::setprecision(1) << relocPct << "\\% ";
  }

  std::cout << "\\\\\n\\cellcolor{white}\\multirow{-2}{*}{" << relocTag
  << "} & \\cellcolor{" << icpColor << "} + ICP ";
  for (auto sequence : sequenceNames)
  {
    auto seqResult = results[sequence];

    float icpPct = static_cast<float>(seqResult.validPosesAfterICP)
    / static_cast<float>(seqResult.poseCount) * 100.f;
    float icpColorPct = (icpPct / 100.f) * (colorMax - colorMin) + colorMin;

    std::cout << "& \\cellcolor{" << icpColor << "!" << icpColorPct << "} "
    << std::setprecision(1) << icpPct << "\\% ";
  }

  std::cout << "\\\\\n";
#endif

  return 0;
}
