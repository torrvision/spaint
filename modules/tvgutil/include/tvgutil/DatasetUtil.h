/**
 * tvgutil: DatasetUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_DATASETUTIL
#define H_TVGUTIL_DATASETUTIL

#include <cstdlib>
#include <string>
#include <vector>

namespace tvgutil {

/**
 * \brief This struct contains utility functions for working with datasets.
 */
struct DatasetUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Checks if a list of paths exist and returns the number of missing paths.
   *
   * \param paths  The paths to check.
   * \return       The number of missing paths.
   */
  static size_t check_paths_exist(const std::vector<std::string>& paths);

  /**
   * \brief Checks if a path exists and outputs the missing paths.
   *
   * \param path  The path to check.
   */
  static bool check_path_exists(const std::string& path);
};

}

#endif
