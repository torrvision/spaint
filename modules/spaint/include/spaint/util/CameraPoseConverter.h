/**
 * spaint: CameraPoseConverter.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_CAMERAPOSECONVERTER
#define H_SPAINT_CAMERAPOSECONVERTER

#include <ORUtils/SE3Pose.h>

#include <rigging/SimpleCamera.h>

namespace spaint {

/**
 * \brief This class contains helper functions to allow us to convert between cameras, InfiniTAM poses and model-view matrices.
 */
struct CameraPoseConverter
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Converts a camera to an InfiniTAM pose.
   *
   * \param camera  The camera.
   * \return        The pose of the camera.
   */
  static ORUtils::SE3Pose camera_to_pose(const rigging::Camera& camera);

  /**
   * \brief Converts an InfiniTAM pose to a camera.
   *
   * \param pose  The pose.
   * \return      A camera with the specified pose.
   */
  static rigging::SimpleCamera pose_to_camera(const ORUtils::SE3Pose& pose);

  /**
   * \brief Converts an InfiniTAM pose to a model-view matrix.
   *
   * \param pose  The pose.
   * \return      The model-view matrix.
   */
  static Eigen::Matrix4f pose_to_modelview(const ORUtils::SE3Pose& pose);
};

}

#endif
