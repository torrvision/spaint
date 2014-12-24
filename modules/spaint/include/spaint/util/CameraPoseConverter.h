/**
 * spaint: CameraPoseConverter.h
 */

#ifndef H_SPAINT_CAMERAPOSECONVERTER
#define H_SPAINT_CAMERAPOSECONVERTER

#include <ITMLib/Objects/ITMPose.h>

#include <rigging/Camera.h>

namespace spaint {

struct CameraPoseConverter
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Converts a camera to an InfiniTAM pose.
   *
   * \param camera  The camera.
   * \return        The InfiniTAM pose of the camera.
   */
  static ITMLib::Objects::ITMPose camera_to_pose(const rigging::Camera& camera);
};

}

#endif
