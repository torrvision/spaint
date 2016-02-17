/**
 * tvgutil: DirectoryUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_DIRECTORYUTIL
#define H_TVGUTIL_DIRECTORYUTIL

#include <string>
#include <vector>

namespace tvgutil {

/**
 * \brief This struct contains utility functions for working with directories.
 */
struct DirectoryUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Creates the directories specified by a list of paths.
   *
   * \param dirs  The directories to create.
   */
  static void create_directories(const std::vector<std::string>& dirs);

  /**
   * \brief Gets the number of files (including directories) contained in the specified directory.
   *
   * \param dir The path to the directory.
   * \return    The number of files (including directories) contained in the specified directory.
   */
  static size_t get_file_count(const std::string& dir);
};

}

#endif
