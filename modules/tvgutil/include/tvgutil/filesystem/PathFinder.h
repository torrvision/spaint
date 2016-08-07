/**
 * tvgutil: PathFinder.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_PATHFINDER
#define H_TVGUTIL_PATHFINDER

#include <boost/filesystem.hpp>

namespace tvgutil {

/**
 * \brief Finds the path to the current executable.
 *
 * \return  The path to the current executable.
 */
extern boost::filesystem::path find_executable();

/**
 * \brief Finds the path to a named subdirectory of the directory containing the current executable.
 *
 * \param name  The name of the subdirectory whose path we want to get.
 * \return      The path to the subdirectory with the specified name.
 */
extern boost::filesystem::path find_subdir_from_executable(const std::string& name);

}

#endif
