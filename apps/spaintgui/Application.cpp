/**
 * spaintgui: Application.cpp
 */

#include "Application.h"

#include <stdexcept>

#include <spaint/ogl/WrappedGL.h>
using namespace spaint;

#ifdef WITH_OVR
#include "RiftRenderer.h"
#endif
#include "WindowedRenderer.h"

//#################### CONSTRUCTORS ####################

Application::Application(const spaint::SpaintEngine_Ptr& spaintEngine)
: m_spaintEngine(spaintEngine)
{
  m_renderer.reset(new WindowedRenderer(spaintEngine, "Semantic Paint", 640, 480));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Application::run()
{
  for(;;)
  {
    if(!process_events() || m_inputState.key_down(SDLK_ESCAPE)) return;

    // Switch renderers if the user requests it.
    static int framesTillSwitchAllowed = 0;
    const int SWITCH_DELAY = 20;
    if(framesTillSwitchAllowed == 0)
    {
      if(m_inputState.key_down(SDLK_w) && !m_inputState.key_down(SDLK_r))
      {
        m_renderer.reset(new WindowedRenderer(m_spaintEngine, "Semantic Paint", 640, 480));
        framesTillSwitchAllowed = SWITCH_DELAY;
      }
      else if(m_inputState.key_down(SDLK_r) && !m_inputState.key_down(SDLK_w))
      {
#if WITH_OVR
        try
        {
          m_renderer.reset(new RiftRenderer(m_spaintEngine, "Semantic Paint", RiftRenderer::FULLSCREEN_MODE));
          framesTillSwitchAllowed = SWITCH_DELAY;
        }
        catch(std::runtime_error&) {}
#endif
      }
    }
    else --framesTillSwitchAllowed;

    // Process and render the next frame.
    m_spaintEngine->process_frame();
    m_renderer->render();

#if 0
    for(int i = 0; i < 6; ++i)
    {
      std::cout << m_spaintEngine->get_pose().params.all[i] << ' ';
    }
    std::cout << '\n';
#endif
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
