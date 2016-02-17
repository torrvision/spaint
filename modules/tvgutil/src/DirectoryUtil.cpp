/**
 * tvgutil: DirectoryUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "DirectoryUtil.h"

#include <stdexcept>

#include <boost/filesystem.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void DirectoryUtil::create_directories(const std::vector<std::string>& dirs)
{
  for(std::vector<std::string>::const_iterator it = dirs.begin(), iend = dirs.end(); it != iend; ++it)
  {
    if(!boost::filesystem::exists(*it)) boost::filesystem::create_directory(*it);
  }
}

size_t DirectoryUtil::get_file_count(const std::string& dir)
{
  if(!boost::filesystem::is_directory(dir))
  {
    throw std::runtime_error("Directory not found: " + dir);
  }

  size_t fileCount = 0;
  for(boost::filesystem::directory_iterator it(dir), iend = boost::filesystem::directory_iterator(); it != iend; ++it)
  {
    ++fileCount;
  }

  return fileCount;
}

}
