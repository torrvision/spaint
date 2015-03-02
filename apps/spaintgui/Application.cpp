/**
 * spaintgui: Application.cpp
 */

#include "Application.h"

#include <stdexcept>

#include <rigging/MoveableCamera.h>
using namespace rigging;

#include <spaint/ogl/WrappedGL.h>
using namespace spaint;

#ifdef WITH_OVR
#include "RiftRenderer.h"
#endif
#include "WindowedRenderer.h"

//#################### CONSTRUCTORS ####################

Application::Application(const SpaintPipeline_Ptr& spaintPipeline)
: m_spaintPipeline(spaintPipeline)
{
  m_renderer.reset(new WindowedRenderer(spaintPipeline->get_model(), spaintPipeline->get_raycaster(), "Semantic Paint", 640, 480));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Application::run()
{
  for(;;)
  {
    if(!process_events() || m_inputState.key_down(SDLK_ESCAPE)) return;

    // Take action as relevant based on the current input state.
    process_input();

    // Process and render the next frame.
    m_spaintPipeline->process_frame();
    m_renderer->render(m_spaintPipeline->get_interactor()->get_selector());
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void Application::handle_key_down(const SDL_Keysym& keysym)
{
  m_inputState.press_key(keysym.sym);

  // If the F key is pressed, toggle whether or not fusion is run as part of the pipeline.
  if(keysym.sym == SDLK_f)
  {
    m_spaintPipeline->set_fusion_enabled(!m_spaintPipeline->get_fusion_enabled());
  }

  // If the H key is pressed, print out a list of keyboard controls.
  if(keysym.sym == SDLK_h)
  {
    std::cout << "\nControls:\n\n"
              << "W = Forwards\n"
              << "S = Backwards\n"
              << "A = Strafe Left\n"
              << "D = Strafe Right\n"
              << "Q = Move Up\n"
              << "E = Move Down\n"
              << "F = Toggle Fusion\n"
              << "Up = Look Down\n"
              << "Down = Look Up\n"
              << "Left = Turn Left\n"
              << "Right = Turn Right\n"
              << "R + 1 = To Windowed Renderer\n"
              << "R + 2 = To Rift Renderer (Windowed)\n"
              << "R + 3 = To Rift Renderer (Fullscreen)\n"
              << "V + 1 = To Follow Camera Mode\n"
              << "V + 2 = To Free Camera Mode\n";
  }
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

void Application::process_camera_input()
{
  // Allow the user to switch camera modes.
  if(m_inputState.key_down(SDLK_v))
  {
    if(m_inputState.key_down(SDLK_1)) m_renderer->set_camera_mode(Renderer::CM_FOLLOW);
    else if(m_inputState.key_down(SDLK_2)) m_renderer->set_camera_mode(Renderer::CM_FREE);
  }

  // If we're in free camera mode, allow the user to move the camera around.
  if(m_renderer->get_camera_mode() == Renderer::CM_FREE)
  {
    const float SPEED = 0.1f;
    const float ANGULAR_SPEED = 0.05f;
    static const Eigen::Vector3f UP(0.0f, -1.0f, 0.0f);

    MoveableCamera_Ptr camera = m_renderer->get_camera();

    if(m_inputState.key_down(SDLK_w)) camera->move_n(SPEED);
    if(m_inputState.key_down(SDLK_s)) camera->move_n(-SPEED);
    if(m_inputState.key_down(SDLK_d)) camera->move_u(-SPEED);
    if(m_inputState.key_down(SDLK_a)) camera->move_u(SPEED);
    if(m_inputState.key_down(SDLK_q)) camera->move(UP, SPEED);
    if(m_inputState.key_down(SDLK_e)) camera->move(UP, -SPEED);

    if(m_inputState.key_down(SDLK_RIGHT)) camera->rotate(UP, -ANGULAR_SPEED);
    if(m_inputState.key_down(SDLK_LEFT)) camera->rotate(UP, ANGULAR_SPEED);
    if(m_inputState.key_down(SDLK_UP)) camera->rotate(camera->u(), ANGULAR_SPEED);
    if(m_inputState.key_down(SDLK_DOWN)) camera->rotate(camera->u(), -ANGULAR_SPEED);
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

void Application::process_input()
{
  process_camera_input();
  process_labelling_input();
  process_renderer_input();
}

void Application::process_labelling_input()
{
  // Allow the user to change the current semantic label.
  static bool canChangeLabel = true;
  const SpaintInteractor_Ptr& interactor = m_spaintPipeline->get_interactor();
  unsigned char semanticLabel = interactor->get_semantic_label();

  if(m_inputState.key_down(SDLK_PAGEUP))
  {
    // FIXME: The maximum semantic label shouldn't be hard-coded like this.
    if(canChangeLabel && semanticLabel < 3) ++semanticLabel;
    canChangeLabel = false;
  }
  else if(m_inputState.key_down(SDLK_PAGEDOWN))
  {
    // FIXME: The minimum semantic label shouldn't be hard-coded like this.
    if(canChangeLabel && semanticLabel > 0) --semanticLabel;
    canChangeLabel = false;
  }
  else canChangeLabel = true;

  interactor->set_semantic_label(semanticLabel);

  // If we're rendering in stereo (e.g. on the Rift), early out.
  if(!m_renderer->get_monocular_render_state()) return;

  // Otherwise, get an appropriate render state for the current camera mode.
  Renderer::RenderState_CPtr renderState;
  switch(m_renderer->get_camera_mode())
  {
    case Renderer::CM_FOLLOW:
      renderState = m_spaintPipeline->get_raycaster()->get_live_render_state();
      break;
    case Renderer::CM_FREE:
      renderState = m_renderer->get_monocular_render_state();
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Unknown camera mode");
  }

  // Update the voxel selector.
  interactor->update_selector(m_inputState, renderState);

  // If the selector is active:
  if(m_inputState.mouse_button_down(MOUSE_BUTTON_LEFT))
  {
    // Determine the voxels selected by the user (if any).
    Selector::Selection_CPtr selection = interactor->select_voxels();

    // If there are selected voxels, mark the voxels with the current semantic label.
    if(selection) interactor->mark_voxels(selection, semanticLabel);
  }
}

void Application::process_renderer_input()
{
  // Allow the user to switch renderers.
  static int framesTillSwitchAllowed = 0;
  const int SWITCH_DELAY = 20;
  if(framesTillSwitchAllowed == 0)
  {
    if(m_inputState.key_down(SDLK_r))
    {
      if(m_inputState.key_down(SDLK_1))
      {
        m_renderer.reset(new WindowedRenderer(m_spaintPipeline->get_model(), m_spaintPipeline->get_raycaster(), "Semantic Paint", 640, 480));
        framesTillSwitchAllowed = SWITCH_DELAY;
      }
      else if(m_inputState.key_down(SDLK_2) || m_inputState.key_down(SDLK_3))
      {
#ifdef WITH_OVR
        try
        {
          m_renderer.reset(new RiftRenderer(
            m_spaintPipeline->get_model(),
            m_spaintPipeline->get_raycaster(),
            "Semantic Paint",
            m_inputState.key_down(SDLK_2) ? RiftRenderer::WINDOWED_MODE : RiftRenderer::FULLSCREEN_MODE
          ));
          framesTillSwitchAllowed = SWITCH_DELAY;
        }
        catch(std::runtime_error&) {}
#endif
      }
    }
  }
  else --framesTillSwitchAllowed;
}
