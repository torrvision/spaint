/**
 * itmx: PosePersister.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "PosePersister.h"

#include <fstream>
#include <stdexcept>

#include <tvgutil/misc/ThreadPool.h>
using tvgutil::ThreadPool;

namespace bf = boost::filesystem;

namespace itmx {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void PosePersister::save_pose(const Matrix4f& pose, const std::string& path)
{
  // Attempt to open the output file.
  std::ofstream fs(path.c_str());
  if(!fs) throw std::runtime_error("Could not open output file: " + path);

  // Write the matrix to the file. Note that we avoid using the matrix-level operator<<
  // because it adds commas between the matrix entries.
  for (int y = 0; y < 4; ++y)
  {
    fs << pose(0, y) << ' ' << pose(1, y) << ' ' << pose(2, y) << ' ' << pose(3, y) << '\n';
  }
}

void PosePersister::save_pose(const Matrix4f& pose, const bf::path& path)
{
  save_pose(pose, path.string());
}

void PosePersister::save_pose_on_thread(const Matrix4f& pose, const std::string& path)
{
  // Select the save_pose overload that takes a string.
  void (*f)(const Matrix4f&, const std::string&) = &save_pose;

  // Call it on a separate thread.
  ThreadPool::instance().post_task(boost::bind(f, pose, path));
}

void PosePersister::save_pose_on_thread(const Matrix4f& pose, const bf::path& path)
{
  save_pose_on_thread(pose, path.string());
}

}
