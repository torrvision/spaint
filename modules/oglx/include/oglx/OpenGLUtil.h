/**
 * oglx: OpenGLUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#ifndef H_OGLX_OPENGLUTIL
#define H_OGLX_OPENGLUTIL

#include "WrappedGL.h"

namespace oglx {

/**
 * \brief This struct provides OpenGL-related helper functions.
 */
struct OpenGLUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Sets appropriate projection and model-view matrices for 2D rendering.
   */
  static void begin_2d();

  /**
   * \brief Restores the projection and model-view matrices that were active prior to 2D rendering.
   */
  static void end_2d();

  /**
   * \brief Renders a quad textured with the specified texture over the top of the existing window contents.
   *
   * \param textureID The ID of the texture to apply to the quad.
   */
  static void render_textured_quad(GLuint textureID);
};

}

#endif
