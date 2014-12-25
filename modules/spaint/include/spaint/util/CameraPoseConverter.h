/**
 * spaint: CameraPoseConverter.h
 */

#ifndef H_SPAINT_CAMERAPOSECONVERTER
#define H_SPAINT_CAMERAPOSECONVERTER

#include <ITMLib/Objects/ITMPose.h>

#include <rigging/SimpleCamera.h>

namespace spaint {

struct CameraPoseConverter
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Converts a camera to an InfiniTAM pose.
   *
   * \param camera  The camera.
   * \return        The pose of the camera.
   */
  static ITMLib::Objects::ITMPose camera_to_pose(const rigging::Camera& camera);

  /**
   * \brief Converts an InfiniTAM pose to a camera.
   *
   * \param pose  The pose.
   * \return      A camera with the specified pose.
   */
  static rigging::SimpleCamera pose_to_camera(const ITMLib::Objects::ITMPose& pose);
};

}

#endif
