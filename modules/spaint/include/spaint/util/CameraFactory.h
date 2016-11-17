/**
 * spaint: CameraFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_CAMERAFACTORY
#define H_SPAINT_CAMERAFACTORY

#include <rigging/SimpleCamera.h>

namespace spaint {

/**
 * \brief This struct provides utility functions to make cameras.
 */
struct CameraFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a default camera.
   */
  template <typename CameraType = rigging::SimpleCamera>
  static boost::shared_ptr<CameraType> make_default_camera()
  {
    return boost::shared_ptr<CameraType>(new CameraType(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));
  }
};

}

#endif
