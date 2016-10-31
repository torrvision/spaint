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
};

SequenceResults evaluate_sequence(const fs::path &gtFolder,
    const fs::path &relocFolder)
{
  SequenceResults res;

  while (true)
  {
    const fs::path gtPath = generate_path(gtFolder, "posem%06i.txt",
        res.poseCount);
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
    const Eigen::Matrix4f finalPose = read_pose_from_file(finalPath);

    bool validReloc = pose_matches(gtPose, relocPose);
    bool validICP = pose_matches(gtPose, icpPose);
    bool validFinal = pose_matches(gtPose, finalPose);

//    std::cout << poseCount << "-> Reloc: " << std::boolalpha << validReloc
//        << " - ICP: " << validICP << std::noboolalpha << '\n';

    ++res.poseCount;
    res.validPosesAfterReloc += validReloc;
    res.validPosesAfterICP += validICP;
    res.validFinalPoses += validFinal;

//    break;
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
        << " \"GT base folder\" \"reloc base output folder\" \"reloc tag\""
        << std::endl;
    return 1;
  }

  fs::path gtFolder = argv[1];
  fs::path relocBaseFolder = argv[2];
  std::string relocTag = argv[3];

  std::vector<std::string> sequenceNames
  { "chess", "fire", "heads", "office", "pumpkin", "redkitchen", "stairs" };
  std::map<std::string, SequenceResults> results;

  for (auto sequence : sequenceNames)
  {
    const fs::path gtPath = gtFolder / sequence / "Test" / "merged";
    const fs::path relocFolder = relocBaseFolder / (relocTag + '_' + sequence);

    std::cerr << "Processing sequence " << sequence << " in:\n\t" << gtPath
        << "\n\t" << relocFolder << std::endl;
    results[sequence] = evaluate_sequence(gtPath, relocFolder);
  }

  // Print table
  printWidth("Sequence", 15, true);
  printWidth("Poses", 8);
  printWidth("Reloc", 8);
  printWidth("ICP", 8);
  printWidth("Final", 8);
  std::cout << '\n';

  for (auto sequence : sequenceNames)
  {
    auto seqResult = results[sequence];

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

  return 0;
}
