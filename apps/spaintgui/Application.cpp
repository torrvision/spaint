/**
 * spaintgui: Application.cpp
 */

#include "Application.h"

#include <spaint/ogl/WrappedGL.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

Application::Application(const spaint::SpaintEngine_Ptr& spaintEngine)
: m_spaintEngine(spaintEngine)
{
  start_rendering_on(RENDERTARGET_WINDOWED);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Application::run()
{
  for(;;)
  {
    if(!process_events() || m_inputState.key_down(SDLK_ESCAPE)) return;
    m_spaintEngine->process_frame();
    render();
    for(int i = 0; i < 6; ++i)
    {
      std::cout << m_spaintEngine->get_pose().params.all[i] << ' ';
    }
    std::cout << '\n';
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void Application::handle_key_down(const SDL_Keysym& keysym)
{
  m_inputState.press_key(keysym.sym);
}

void Application::handle_key_up(const SDL_Keysym& keysym)
{
  m_inputState.release_key(keysym.sym);
}

void Application::handle_mousebutton_down(const SDL_MouseButtonEvent& e)
{
  switch(e.button)
  {
    case SDL_BUTTON_LEFT:
      m_inputState.press_mouse_button(MOUSE_BUTTON_LEFT, e.x, e.y);
      break;
    case SDL_BUTTON_MIDDLE:
      m_inputState.press_mouse_button(MOUSE_BUTTON_MIDDLE, e.x, e.y);
      break;
    case SDL_BUTTON_RIGHT:
      m_inputState.press_mouse_button(MOUSE_BUTTON_RIGHT, e.x, e.y);
      break;
    default:
      break;
  }
}

void Application::handle_mousebutton_up(const SDL_MouseButtonEvent& e)
{
  switch(e.button)
  {
    case SDL_BUTTON_LEFT:
      m_inputState.release_mouse_button(MOUSE_BUTTON_LEFT);
      break;
    case SDL_BUTTON_MIDDLE:
      m_inputState.release_mouse_button(MOUSE_BUTTON_MIDDLE);
      break;
    case SDL_BUTTON_RIGHT:
      m_inputState.release_mouse_button(MOUSE_BUTTON_RIGHT);
      break;
    default:
      break;
  }
}

bool Application::process_events()
{
  SDL_Event event;
  while(SDL_PollEvent(&event))
  {
    switch(event.type)
    {
      case SDL_KEYDOWN:
        handle_key_down(event.key.keysym);
        break;
      case SDL_KEYUP:
        handle_key_up(event.key.keysym);
        break;
      case SDL_MOUSEBUTTONDOWN:
        handle_mousebutton_down(event.button);
        break;
      case SDL_MOUSEBUTTONUP:
        handle_mousebutton_up(event.button);
        break;
      case SDL_MOUSEMOTION:
        m_inputState.set_mouse_position(event.motion.x, event.motion.y);
        m_inputState.set_mouse_motion(event.motion.xrel, event.motion.yrel);
        break;
      case SDL_QUIT:
        return false;
      default:
        break;
    }
  }

  return true;
}

void Application::render() const
{
  static spaint::shared_ptr<ITMUChar4Image> rgbImage(new ITMUChar4Image(m_spaintEngine->get_image_source_engine()->getDepthImageSize(), false));
  static GLuint textureID;
  static bool done = false;
  if(!done)
  {
    glGenTextures(1, &textureID);
    done = true;
  }

#if 1
  m_spaintEngine->get_default_raycast(rgbImage);
#else
  ITMPose pose = m_spaintEngine->get_pose();
  pose.params.each.tx = 0;
  pose.SetModelViewFromParams();
  m_spaintEngine->generate_free_raycast(rgbImage, pose);
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

void Application::start_rendering_on(RenderTarget renderTarget)
{
  switch(renderTarget)
  {
    case RENDERTARGET_WINDOWED:
    {
      const int width = 640;
      const int height = 480;

      SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

      m_window.reset(
        SDL_CreateWindow(
          "Semantic Paint",
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
      break;
    }
#ifdef WITH_OVR
    case RENDERTARGET_RIFT:
    {
      // TODO
      break;
    }
#endif
  }
}

void Application::stop_rendering_on(RenderTarget renderTarget)
{
  // TODO
}

void Application::switch_render_target(RenderTarget renderTarget)
{
  stop_rendering_on(m_renderTarget);
  m_renderTarget = renderTarget;
  start_rendering_on(m_renderTarget);
}
