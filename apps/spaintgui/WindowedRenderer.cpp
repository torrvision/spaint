/**
 * spaintgui: WindowedRenderer.cpp
 */

#include "WindowedRenderer.h"

#include <spaint/ogl/WrappedGL.h>

//#################### CONSTRUCTORS ####################

WindowedRenderer::WindowedRenderer(const std::string& title, int width, int height)
{
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  m_window.reset(
    SDL_CreateWindow(
      title.c_str(),
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      width,
      height,
      SDL_WINDOW_OPENGL
    ),
    &SDL_DestroyWindow
  );

  m_context.reset(
    SDL_GL_CreateContext(m_window.get()),
    SDL_GL_DeleteContext
  );

  glViewport(0, 0, width, height);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void WindowedRenderer::render(const spaint::SpaintEngine_Ptr& spaintEngine) const
{
  static spaint::shared_ptr<ITMUChar4Image> rgbImage(new ITMUChar4Image(spaintEngine->get_image_source_engine()->getDepthImageSize(), false));
  static GLuint textureID;
  static bool done = false;
  if(!done)
  {
    glGenTextures(1, &textureID);
    done = true;
  }

#if 1
  spaintEngine->get_default_raycast(rgbImage);
#else
  ITMPose pose = m_spaintEngine->get_pose();
  pose.params.each.tx = 0;
  pose.SetModelViewFromParams();
  spaintEngine->generate_free_raycast(rgbImage, pose);
#endif

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    glLoadIdentity();
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      glLoadIdentity();

      glEnable(GL_TEXTURE_2D);
      {
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgbImage->noDims.x, rgbImage->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbImage->GetData(false));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glBegin(GL_QUADS);
        {
					glTexCoord2f(0, 1); glVertex2f(0, 0);
					glTexCoord2f(1, 1); glVertex2f(1, 0);
					glTexCoord2f(1, 0); glVertex2f(1, 1);
					glTexCoord2f(0, 0); glVertex2f(0, 1);
        }
				glEnd();
      }
      glDisable(GL_TEXTURE_2D);
    }
    glPopMatrix();
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  SDL_GL_SwapWindow(m_window.get());
}
