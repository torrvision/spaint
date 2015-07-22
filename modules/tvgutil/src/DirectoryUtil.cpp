/**
 * tvgutil: DirectoryUtil.cpp
 */

#include "DirectoryUtil.h"

#include <stdexcept>

#include <boost/filesystem.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

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
