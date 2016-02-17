/**
 * tvgutil: DatasetUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "DatasetUtil.h"

#include <stdexcept>

#include <boost/filesystem.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

size_t DatasetUtil::check_paths_exist(const std::vector<std::string>& paths)
{
  size_t invalidCount = 0;
  for(std::vector<std::string>::const_iterator it = paths.begin(), iend = paths.end(); it != iend; ++it)
  {
    if(!check_path_exists(*it)) ++invalidCount;
  }

  return invalidCount;
}

bool DatasetUtil::check_path_exists(const std::string& path)
{
  if(!boost::filesystem::exists(path))
  {
    std::cout << "Expecting to see: " << path << '\n';
    return false;
  }

  return true;
}

}
