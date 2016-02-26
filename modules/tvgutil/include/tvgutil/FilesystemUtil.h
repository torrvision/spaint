/**
 * tvgutil: FilesystemUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_FILESYSTEMUTIL
#define H_TVGUTIL_FILESYSTEMUTIL

#include <string>
#include <vector>

namespace tvgutil {

/**
 * \brief This struct contains utility functions for working with the file system.
 */
struct FilesystemUtil
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

  /**
   * \brief Checks if a list of paths exist and returns the missing paths.
   *
   * \param paths  The paths to check.
   * \return       The paths that are missing.
   */
  static std::vector<std::string> get_missing_paths(const std::vector<std::string>& paths);
};

}

#endif
