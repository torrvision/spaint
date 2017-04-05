/**
 * spaint: ImagePersister.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "util/PosePersister.h"

#include <fstream>
#include <stdexcept>

#include <ITMLib/Utils/ITMMath.h>

#include <tvgutil/misc/ThreadPool.h>

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void PosePersister::save_pose(const Matrix4f& pose, const std::string& path)
{
  // Open the output file.
  std::ofstream outFile(path.c_str());

  // If the open failed throw an exception.
  if (!outFile) throw std::runtime_error("Could not open output file: " + path);

  // Print the matrix on the file
  // Not using Matrix4f::operator<< because it adds commas between matrix entries.
  for (int y = 0; y < 4; ++y)
  {
    outFile << pose(0, y) << ' ' << pose(1, y) << ' ' << pose(2, y) << ' ' << pose(3, y) << '\n';
  }
}

void PosePersister::save_pose(const Matrix4f& pose, const boost::filesystem::path& path)
{
  save_pose(pose, path.string());
}

void PosePersister::save_pose_on_thread(const Matrix4f& pose, const std::string& path)
{
  // Select the overload taking a string.
  void (*s) (const Matrix4f&, const std::string&) = &save_pose;

  // Call save_pose on a separate thread
  tvgutil::ThreadPool::instance().post_task(boost::bind(s, pose, path));
}

void PosePersister::save_pose_on_thread(const Matrix4f& pose, const boost::filesystem::path& path)
{
  save_pose_on_thread(pose, path.string());
}

}
