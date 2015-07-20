/**
 * spaintgui: Application.cpp
 */

#include "Application.h"

#include <fstream>
#include <stdexcept>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/assign/list_of.hpp>
using boost::assign::map_list_of;

#include <rigging/MoveableCamera.h>
using namespace rigging;

#include <spaint/ogl/WrappedGL.h>
using namespace spaint;

#include <tvgutil/ExecutableFinder.h>
#include <tvgutil/commands/NoOpCommand.h>
using namespace tvgutil;

#ifdef WITH_OVR
#include "RiftRenderer.h"
#endif
#include "WindowedRenderer.h"

#include "commands/MarkVoxelsCommand.h"

//#################### CONSTRUCTORS ####################

Application::Application(const SpaintPipeline_Ptr& spaintPipeline)
: m_commandManager(10),
  m_spaintPipeline(spaintPipeline),
  m_voiceCommandStream("localhost", "23984")
{
  m_renderer.reset(new WindowedRenderer("Semantic Paint", spaintPipeline->get_model(), spaintPipeline->get_raycaster()));
  setup_labels();
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

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::filesystem::path Application::resources_dir()
{
  boost::filesystem::path p = find_executable(); // spaint/build/bin/apps/spaintgui/spaintgui(.exe)
  p = p.parent_path();                           // spaint/build/bin/apps/spaintgui/
  p = p / "resources/";                          // spaint/build/bin/apps/spaintgui/resources/
  return p;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

Application::RenderState_CPtr Application::get_monocular_render_state() const
{
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

  // If the backspace key is pressed, clear the semantic labels of all the voxels in the scene, and reset the random forest and command manager.
  if(keysym.sym == SDLK_BACKSPACE)
  {
    m_spaintPipeline->get_interactor()->clear_labels();
    m_spaintPipeline->reset_forest();
    m_commandManager.reset();
  }

  // If the hash key is pressed, toggle whether or not median filtering is used when rendering the scene raycast.
  if(keysym.sym == SDLK_HASH)
  {
    m_renderer->set_median_filtering_enabled(!m_renderer->get_median_filtering_enabled());
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
              << "C + 1 = To Semantic Lambertian Raycast\n"
              << "C + 2 = To Semantic Phong Raycast\n"
              << "C + 3 = To Semantic Colour Raycast\n"
              << "I + 1 = To Null Selector\n"
              << "I + 2 = To Picking Selector\n"
              << "I + 3 = To Leap Selector\n"
              << "I + 4 = To Touch Selector\n"
              << "M + 1 = To Normal Mode\n"
              << "M + 2 = To Training Mode\n"
              << "M + 3 = To Prediction Mode\n"
              << "M + 4 = To Propagation Mode\n"
              << "M + 5 = To Feature Inspection Mode\n"
              << "R + 1 = To Windowed Renderer\n"
              << "R + 2 = To Rift Renderer (Windowed)\n"
              << "R + 3 = To Rift Renderer (Fullscreen)\n"
              << "V + 1 = To Follow Camera Mode\n"
              << "V + 2 = To Free Camera Mode\n"
              << "[ = Decrease Picking Selection Radius\n"
              << "] = Increase Picking Selection Radius\n"
              << "RShift + [ = To Previous Semantic Label\n"
              << "RShift + ] = To Next Semantic Label\n"
              << "Backspace = Reset (Clear Labels and Forest)\n"
              << "# = Toggle Median Filtering\n";
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
  process_voice_input();
}

void Application::process_labelling_input()
{
  // Allow the user to change the current semantic label.
  static bool canChangeLabel = true;
  const SpaintInteractor_Ptr& interactor = m_spaintPipeline->get_interactor();
  LabelManager_CPtr labelManager = m_spaintPipeline->get_model()->get_label_manager();
  SpaintVoxel::Label semanticLabel = interactor->get_semantic_label();

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
  interactor->update_selector(m_inputState, get_monocular_render_state(), m_renderer->is_mono());

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
      const SpaintVoxel::PackedLabel packedLabel(semanticLabel, SpaintVoxel::LG_USER);
      const bool useUndo = true;
      if(useUndo)
      {
        if(!currentlyMarking)
        {
          m_commandManager.execute_command(Command_CPtr(new NoOpCommand(beginMarkVoxelsDesc)));
          currentlyMarking = true;
        }
        m_commandManager.execute_compressible_command(Command_CPtr(new MarkVoxelsCommand(selection, packedLabel, interactor)), precursors);
      }
      else interactor->mark_voxels(selection, packedLabel);
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
    else if(m_inputState.key_down(SDLK_4)) mode = SpaintPipeline::MODE_PROPAGATION;
    else if(m_inputState.key_down(SDLK_5)) mode = SpaintPipeline::MODE_FEATURE_INSPECTION;
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
        m_renderer.reset(new WindowedRenderer("Semantic Paint", m_spaintPipeline->get_model(), m_spaintPipeline->get_raycaster()));
        framesTillSwitchAllowed = SWITCH_DELAY;
      }
      else if(m_inputState.key_down(SDLK_2) || m_inputState.key_down(SDLK_3))
      {
#ifdef WITH_OVR
        try
        {
          m_renderer.reset(new RiftRenderer(
            "Semantic Paint",
            m_spaintPipeline->get_model(),
            m_spaintPipeline->get_raycaster(),
            m_inputState.key_down(SDLK_2) ? RiftRenderer::WINDOWED_MODE : RiftRenderer::FULLSCREEN_MODE
          ));
          framesTillSwitchAllowed = SWITCH_DELAY;
        }
        catch(std::runtime_error& e)
        {
          std::cerr << e.what() << '\n';
        }
#endif
      }
    }
  }
  else --framesTillSwitchAllowed;

  // Allow the user to switch raycast type.
  if(m_inputState.key_down(SDLK_c))
  {
    if(m_inputState.key_down(SDLK_1)) m_renderer->set_raycast_type(SpaintRaycaster::RT_SEMANTICLAMBERTIAN);
    else if(m_inputState.key_down(SDLK_2)) m_renderer->set_raycast_type(SpaintRaycaster::RT_SEMANTICPHONG);
    else if(m_inputState.key_down(SDLK_3)) m_renderer->set_raycast_type(SpaintRaycaster::RT_SEMANTICCOLOUR);
  }
}

void Application::process_voice_input()
{
  // If we are not connected to a voice command server, early out.
  if(!m_voiceCommandStream) return;

  size_t availableBytes;
  while((availableBytes = m_voiceCommandStream.rdbuf()->available()) > 0)
  {
    // If there is a voice command available, get it from the stream and trim it to remove any trailing carriage return.
    std::string command;
    std::getline(m_voiceCommandStream, command);
    boost::trim(command);

    // Output the voice command for debugging purposes.
    std::cout << "Voice Command: " << command << '\n';

    // Process any requests to change label.
    const LabelManager_Ptr& labelManager = m_spaintPipeline->get_model()->get_label_manager();
    for(size_t i = 0, labelCount = labelManager->get_label_count(); i < labelCount; ++i)
    {
      SpaintVoxel::Label label = static_cast<SpaintVoxel::Label>(i);
      std::string changeLabelCommand = "label " + labelManager->get_label_name(label);
      if(command == changeLabelCommand) m_spaintPipeline->get_interactor()->set_semantic_label(label);
    }

    // Process any requests to disable/enable fusion.
    if(command == "disable fusion") m_spaintPipeline->set_fusion_enabled(false);
    if(command == "enable fusion") m_spaintPipeline->set_fusion_enabled(true);

    // Process any requests to change pipeline mode.
    if(command == "switch to normal mode") m_spaintPipeline->set_mode(SpaintPipeline::MODE_NORMAL);
    if(command == "switch to prediction mode") m_spaintPipeline->set_mode(SpaintPipeline::MODE_PREDICTION);
    if(command == "switch to training mode") m_spaintPipeline->set_mode(SpaintPipeline::MODE_TRAINING);
  }
}

void Application::setup_labels()
{
  const LabelManager_Ptr& labelManager = m_spaintPipeline->get_model()->get_label_manager();
  std::ifstream fs((resources_dir() / "Labels.txt").c_str());
  if(fs)
  {
    // If a labels file is present, load the labels from it.
    std::cout << "[spaint] Loading labels...\n";

    std::string label;
    while(std::getline(fs, label))
    {
      boost::trim(label);
      if(label != "") labelManager->add_label(label);
    }

    // Add additional dummy labels up to the maximum number of labels we are allowed.
    for(size_t i = labelManager->get_label_count(), count = labelManager->get_max_label_count(); i < count; ++i)
    {
      labelManager->add_label(boost::lexical_cast<std::string>(i));
    }
  }
  else
  {
    // Otherwise, use a set of dummy labels.
    std::cout << "[spaint] Failed to load labels, reverting to a set of dummy labels...\n";

    labelManager->add_label("background");
    for(size_t i = 1, count = labelManager->get_max_label_count(); i < count; ++i)
    {
      labelManager->add_label(boost::lexical_cast<std::string>(i));
    }
  }

  // Set the initial semantic label to use for painting.
  m_spaintPipeline->get_interactor()->set_semantic_label(1);
}
