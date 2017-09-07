/**
 * spaint: CameraRenderer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_CAMERARENDERER
#define H_SPAINT_CAMERARENDERER

#include <boost/optional.hpp>

#include <ITMLib/Utils/ITMMath.h>

#include <rigging/Camera.h>

namespace spaint {

/**
 * \brief This struct provides functions to render cameras in the scene.
 */
struct CameraRenderer
{
  //#################### ENUMERATIONS ####################

  /**
   * \brief The values of this enumeration denote the types of axes that can be rendered for a camera.
   */
  enum AxesType
  {
    AXES_NUV,
    AXES_XYZ
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Renders a camera in the scene.
   *
   * \param cam         The camera to render.
   * \param axesType    The type of axes to render for the camera.
   * \param axisScale   The scale factor to apply to each axis.
   * \param bodyColour  The colour to use for the camera's body (if we want to render it).
   * \param bodyScale   The scale factor to apply to the camera's body (if we're rendering it).
   */
  static void render_camera(const rigging::Camera& cam, AxesType axesType = AXES_XYZ, float axisScale = 1.0f,
                            const boost::optional<Vector3f>& bodyColour = boost::none, double bodyScale = 0.02);
};

}

#endif
