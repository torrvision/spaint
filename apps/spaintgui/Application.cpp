/**
 * spaintgui: Application.cpp
 */

#include "Application.h"

#include <stdexcept>

#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>
using boost::assign::map_list_of;

#include <rigging/MoveableCamera.h>
using namespace rigging;

#include <spaint/ogl/WrappedGL.h>
using namespace spaint;

#include <tvgutil/commands/NoOpCommand.h>
using namespace tvgutil;

#ifdef WITH_OVR
#include "RiftRenderer.h"
#endif
#include "WindowedRenderer.h"

#include "commands/MarkVoxelsCommand.h"

//#################### CONSTRUCTORS ####################

Application::Application(const SpaintPipeline_Ptr& spaintPipeline)
: m_commandManager(10), m_spaintPipeline(spaintPipeline)
{
  m_renderer.reset(new WindowedRenderer(spaintPipeline->get_model(), spaintPipeline->get_raycaster(), "Semantic Paint"));

  // Set up the semantic labels.
  const LabelManager_Ptr& labelManager = m_spaintPipeline->get_model()->get_label_manager();
  labelManager->add_label("Background");
  for(size_t i = 1, count = labelManager->get_max_label_count(); i < count; ++i)
  {
    labelManager->add_label(boost::lexical_cast<std::string>(i));
  }

  // Set the initial semantic label to use for painting.
  m_spaintPipeline->get_interactor()->set_semantic_label(1);
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
    m_spaintPipeline->run_main_section();
    m_renderer->render(m_spaintPipeline->get_interactor());

    // Run the mode-specific section of the pipeline.
    m_spaintPipeline->run_mode_specific_section(get_monocular_render_state());
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

Application::RenderState_CPtr Application::get_monocular_render_state() const
{
  // If we're rendering in stereo (e.g. on the Rift), return a null render state.
  if(!m_renderer->get_monocular_render_state()) return RenderState_CPtr();

  // Otherwise, return the monocular render state corresponding to the current camera mode.
  switch(m_renderer->get_camera_mode())
  {
    case Renderer::CM_FOLLOW:
      return m_spaintPipeline->get_raycaster()->get_live_render_state();
    case Renderer::CM_FREE:
      return m_renderer->get_monocular_render_state();
    default:
      // This should never happen.
      throw std::runtime_error("Unknown camera mode");
  }
}

void Application::handle_key_down(const SDL_Keysym& keysym)
{
  m_inputState.press_key(keysym.sym);

  // If the F key is pressed, toggle whether or not fusion is run as part of the pipeline.
  if(keysym.sym == SDLK_f)
  {
    m_spaintPipeline->set_fusion_enabled(!m_spaintPipeline->get_fusion_enabled());
  }

  // If the P key is pressed, toggle whether or not Phong lighting is enabled.
  if(keysym.sym == SDLK_p)
  {
    m_renderer->set_phong_enabled(!m_renderer->get_phong_enabled());
  }

  // If the backspace key is pressed, clear the semantic labels of all the voxels in the scene and reset the command manager.
  if(keysym.sym == SDLK_BACKSPACE)
  {
    m_spaintPipeline->get_interactor()->clear_labels();
    m_commandManager.reset();
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
              << "P = Toggle Phong Lighting\n"
              << "Up = Look Down\n"
              << "Down = Look Up\n"
              << "Left = Turn Left\n"
              << "Right = Turn Right\n"
              << "I + 1 = To Null Selector\n"
              << "I + 2 = To Picking Selector\n"
              << "I + 3 = To Leap Selector\n"
              << "M + 1 = To Normal Mode\n"
              << "M + 2 = To Training Mode\n"
              << "M + 3 = To Prediction Mode\n"
              << "R + 1 = To Windowed Renderer\n"
              << "R + 2 = To Rift Renderer (Windowed)\n"
              << "R + 3 = To Rift Renderer (Fullscreen)\n"
              << "V + 1 = To Follow Camera Mode\n"
              << "V + 2 = To Free Camera Mode\n"
              << "[ = Decrease Picking Selection Radius\n"
              << "] = Increase Picking Selection Radius\n"
              << "RShift + [ = To Previous Semantic Label\n"
              << "RShift + ] = To Next Semantic Label\n"
              << "Backspace = Clear Semantic Labels\n";
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

void Application::process_command_input()
{
  static bool blockUndo = false;
  if(m_inputState.key_down(SDLK_LCTRL) && m_inputState.key_down(SDLK_z))
  {
    if(!blockUndo && m_commandManager.can_undo())
    {
      m_commandManager.undo();
      blockUndo = true;
    }
  }
  else blockUndo = false;

  static bool blockRedo = false;
  if(m_inputState.key_down(SDLK_LCTRL) && m_inputState.key_down(SDLK_y))
  {
    if(!blockRedo && m_commandManager.can_redo())
    {
      m_commandManager.redo();
      blockRedo = true;
    }
  }
  else blockRedo = false;
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
  process_command_input();
  process_labelling_input();
  process_mode_input();
  process_renderer_input();
}

void Application::process_labelling_input()
{
  // Get the current monocular render state, if any. If we're not currently rendering in mono, early out.
  RenderState_CPtr renderState = get_monocular_render_state();
  if(!renderState) return;

  // Allow the user to change the current semantic label.
  static bool canChangeLabel = true;
  const SpaintInteractor_Ptr& interactor = m_spaintPipeline->get_interactor();
  LabelManager_CPtr labelManager = m_spaintPipeline->get_model()->get_label_manager();
  SpaintVoxel::LabelType semanticLabel = interactor->get_semantic_label();

  if(m_inputState.key_down(SDLK_RSHIFT) && m_inputState.key_down(SDLK_RIGHTBRACKET))
  {
    if(canChangeLabel) semanticLabel = labelManager->get_next_label(semanticLabel);
    canChangeLabel = false;
  }
  else if(m_inputState.key_down(SDLK_RSHIFT) && m_inputState.key_down(SDLK_LEFTBRACKET))
  {
    if(canChangeLabel) semanticLabel = labelManager->get_previous_label(semanticLabel);
    canChangeLabel = false;
  }
  else canChangeLabel = true;

  interactor->set_semantic_label(semanticLabel);

  // Update the current selector.
  interactor->update_selector(m_inputState, renderState);

  // Record whether or not we're in the middle of marking some voxels (this allows us to make voxel marking atomic for undo/redo purposes).
  static bool currentlyMarking = false;

  // Specify the precursors map for compressible commands.
  static const std::string beginMarkVoxelsDesc = "Begin Mark Voxels";
  static const std::string markVoxelsDesc = MarkVoxelsCommand::get_static_description();
  static std::map<std::string,std::string> precursors = map_list_of(beginMarkVoxelsDesc,markVoxelsDesc)(markVoxelsDesc,markVoxelsDesc);

  // If the current selector is active:
  if(interactor->selector_is_active())
  {
    // Get the voxels selected by the user (if any).
    Selector::Selection_CPtr selection = interactor->get_selection();

    // If there are selected voxels, mark the voxels with the current semantic label.
    if(selection)
    {
      const bool useUndo = true;
      if(useUndo)
      {
        if(!currentlyMarking)
        {
          m_commandManager.execute_command(Command_CPtr(new NoOpCommand(beginMarkVoxelsDesc)));
          currentlyMarking = true;
        }
        m_commandManager.execute_compressible_command(Command_CPtr(new MarkVoxelsCommand(selection, semanticLabel, interactor)), precursors);
      }
      else interactor->mark_voxels(selection, semanticLabel);
    }
  }
  else if(currentlyMarking)
  {
    m_commandManager.execute_compressible_command(Command_CPtr(new NoOpCommand("End Mark Voxels")), precursors);
    currentlyMarking = false;
  }
}

void Application::process_mode_input()
{
  SpaintPipeline::Mode mode = m_spaintPipeline->get_mode();
  if(m_inputState.key_down(SDLK_m))
  {
    if(m_inputState.key_down(SDLK_1))      mode = SpaintPipeline::MODE_NORMAL;
    else if(m_inputState.key_down(SDLK_2)) mode = SpaintPipeline::MODE_TRAINING;
    else if(m_inputState.key_down(SDLK_3)) mode = SpaintPipeline::MODE_PREDICTION;
  }
  m_spaintPipeline->set_mode(mode);
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
        m_renderer.reset(new WindowedRenderer(m_spaintPipeline->get_model(), m_spaintPipeline->get_raycaster(), "Semantic Paint"));
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
