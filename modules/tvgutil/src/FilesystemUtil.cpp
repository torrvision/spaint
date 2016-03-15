/**
 * tvgutil: FilesystemUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "FilesystemUtil.h"

#include <stdexcept>

#include <boost/filesystem.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void FilesystemUtil::create_directories(const std::list<std::string>& dirs)
{
  for(std::list<std::string>::const_iterator it = dirs.begin(), iend = dirs.end(); it != iend; ++it)
  {
    if(!boost::filesystem::exists(*it))
    {
      boost::filesystem::create_directory(*it);
    }
  }
}

size_t FilesystemUtil::get_file_count(const std::string& dir)
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

std::list<std::string> FilesystemUtil::get_missing_paths(const std::list<std::string>& paths)
{
  std::list<std::string> missingPaths;

  for(std::list<std::string>::const_iterator it = paths.begin(), iend = paths.end(); it != iend; ++it)
  {
    if(!boost::filesystem::exists(*it))
    {
      missingPaths.push_back(*it);
    }
  }

  return missingPaths;
}

}
