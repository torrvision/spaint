/**
 * spaint: ImagePersister.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "util/PosePersister.h"

#include <fstream>
#include <stdexcept>

#include <boost/thread.hpp>

#include <ITMLib/Utils/ITMMath.h>

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void PosePersister::save_pose(const ORUtils::SE3Pose& pose, const std::string& path)
{
  // Open the output file.
  std::ofstream outFile(path, std::ios::out);

  // If the open failed throw an exception.
  if (!outFile.is_open()) throw std::runtime_error("Could not open output file: " + path);

  // Print the matrix on the file
  // Not using Matrix4f::operator<< because it adds commas between matrix entries.
  const Matrix4f &m = pose.GetM();
  for (int y = 0; y < 4; ++y)
  {
    outFile << m(0, y) << " " << m(1, y) << " " << m(2, y) << " " << m(3, y) << "\n";
  }

  outFile.close();
}

void PosePersister::save_pose(const ORUtils::SE3Pose& pose, const boost::filesystem::path& path)
{
  save_pose(pose, path.string());
}

void PosePersister::save_pose_on_thread(const ORUtils::SE3Pose& pose, const std::string& path)
{
  // Select the overload taking a string.
  void (*s) (const ORUtils::SE3Pose&, const std::string&) = &save_pose;

  // Start a thread and call save_pose(...).
  boost::thread t(s, pose, path);
  t.detach();
}

void PosePersister::save_pose_on_thread(const ORUtils::SE3Pose& pose, const boost::filesystem::path& path)
{
  save_pose_on_thread(pose, path.string());
}

}
