/**
 * itmx: PosePersister.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_ITMX_POSEPERSISTER
#define H_ITMX_POSEPERSISTER

#include <string>

#include <boost/filesystem.hpp>

#include <ITMLib/Utils/ITMMath.h>

namespace itmx {

/**
 * \brief This class contains utility functions for saving camera poses.
 */
class PosePersister
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to save a camera pose to a file.
   *
   * \param pose                The pose matrix to save.
   * \param path                The path to the file to which to save it.
   * \throws std::runtime_error If the pose could not be saved.
   */
  static void save_pose(const Matrix4f& pose, const std::string& path);

  /**
   * \brief Attempts to save a camera pose to a file.
   *
   * \param pose                The pose matrix to save.
   * \param path                The path to the file to which to save it.
   * \throws std::runtime_error If the pose could not be saved.
   */
  static void save_pose(const Matrix4f& pose, const boost::filesystem::path& path);

  /**
   * \brief Attempts to save a camera pose to a file on a separate thread.
   *
   * \param pose                The pose matrix to save.
   * \param path                The path to the file to which to save it.
   * \throws std::runtime_error If the pose could not be saved.
   */
  static void save_pose_on_thread(const Matrix4f& pose, const std::string& path);

  /**
   * \brief Attempts to save a camera pose to a file on a separate thread.
   *
   * \param pose                The pose matrix to save.
   * \param path                The path to the file to which to save it.
   * \throws std::runtime_error If the pose could not be saved.
   */
  static void save_pose_on_thread(const Matrix4f& pose, const boost::filesystem::path& path);
};

}

#endif
