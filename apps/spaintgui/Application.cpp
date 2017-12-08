/**
 * spaintgui: Application.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Application.h"
using namespace tvginput;

#include <fstream>
#include <stdexcept>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/assign/list_of.hpp>
using boost::assign::map_list_of;

#include <ITMLib/Engines/Meshing/ITMMeshingEngineFactory.h>
#include <ITMLib/Objects/Camera/ITMCalibIO.h>
using namespace ITMLib;

#include <itmx/persistence/ImagePersister.h>
#include <itmx/persistence/PosePersister.h>
using namespace itmx;

#include <rigging/MoveableCamera.h>
using namespace rigging;

#include <spaint/ogl/WrappedGL.h>
using namespace spaint;

#include <tvgutil/commands/NoOpCommand.h>
#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/timing/TimeUtil.h>
using namespace tvgutil;

#ifdef WITH_OVR
#include "renderers/RiftRenderer.h"
#endif
#include "renderers/WindowedRenderer.h"

#include "commands/MarkVoxelsCommand.h"

//#################### CONSTRUCTORS ####################

Application::Application(const MultiScenePipeline_Ptr& pipeline, bool renderFiducials)
: m_activeSubwindowIndex(0),
  m_batchModeEnabled(false),
  m_commandManager(10),
  m_pauseBetweenFrames(true),
  m_paused(true),
  m_pipeline(pipeline),
  m_renderFiducials(renderFiducials),
  m_usePoseMirroring(true),
  m_voiceCommandStream("localhost", "23984")
{
  setup_labels();
  setup_meshing();

  const Settings_CPtr& settings = m_pipeline->get_model()->get_settings();
  int subwindowConfigurationIndex = settings->get_first_value<int>("subwindowConfigurationIndex");
  switch_to_windowed_renderer(subwindowConfigurationIndex);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool Application::run()
{
  for(;;)
  {
    // Check to see if the user wants to quit the application, and quit if necessary. Note that if we
    // are running in batch mode, we quit directly, rather than saving a mesh of the scene on exit.
    bool eventQuit = !process_events();
    bool escQuit = m_inputState.key_down(KEYCODE_ESCAPE);
    if(m_batchModeEnabled) { if(eventQuit) return false; }
    else                   { if(eventQuit || escQuit) break; }

    // Take action as relevant based on the current input state.
    process_input();

    // If the application is unpaused, process a new frame.
    if(!m_paused)
    {
      // Run the main section of the pipeline.
      bool frameWasProcessed = m_pipeline->run_main_section();

      if(frameWasProcessed)
      {
        // If a frame debug hook is active, call it.
        if(m_frameDebugHook) m_frameDebugHook(m_pipeline->get_model());

        // If we're currently recording the sequence, save the frame to disk.
        if(m_sequencePathGenerator) save_sequence_frame();
      }
      else if(m_batchModeEnabled)
      {
        // If we're running in batch mode and we reach the end of the sequence, quit.
        break;
      }
    }

    // Render the scene.
    m_renderer->render(m_fracWindowPos, m_renderFiducials);

    // If the application is unpaused, run the mode-specific section of the pipeline for the active scene.
    if(!m_paused) m_pipeline->run_mode_specific_section(get_active_scene_id(), get_monocular_render_state());

    // If we're currently recording a video, save the next frame of it to disk.
    if(m_videoPathGenerator) save_video_frame();

    // If desired, pause at the end of each frame for debugging purposes.
    if(m_pauseBetweenFrames) m_paused = true;
  }

  // If desired, save a mesh of the scene before the application terminates.
  if(m_saveMeshOnExit) save_mesh();

  return true;
}

void Application::set_batch_mode_enabled(bool batchModeEnabled)
{
  m_batchModeEnabled = batchModeEnabled;
  m_paused = m_pauseBetweenFrames = !batchModeEnabled;
}

void Application::set_server_mode_enabled(bool serverModeEnabled)
{
  m_paused = m_pauseBetweenFrames = !serverModeEnabled;
}

void Application::set_frame_debug_hook(const FrameDebugHook& frameDebugHook)
{
  m_frameDebugHook = frameDebugHook;
}

void Application::set_save_mesh_on_exit(bool saveMeshOnExit)
{
  m_saveMeshOnExit = saveMeshOnExit;
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::filesystem::path Application::resources_dir()
{
  return find_subdir_from_executable("resources");
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

const std::string& Application::get_active_scene_id() const
{
  return get_active_subwindow().get_scene_id();
}

Subwindow& Application::get_active_subwindow()
{
  return m_renderer->get_subwindow_configuration()->subwindow(m_activeSubwindowIndex);
}

const Subwindow& Application::get_active_subwindow() const
{
  return m_renderer->get_subwindow_configuration()->subwindow(m_activeSubwindowIndex);
}

VoxelRenderState_CPtr Application::get_monocular_render_state() const
{
  const Subwindow& subwindow = get_active_subwindow();
  switch(subwindow.get_camera_mode())
  {
    case Subwindow::CM_FOLLOW:
      return m_pipeline->get_model()->get_slam_state(subwindow.get_scene_id())->get_live_voxel_render_state();
    case Subwindow::CM_FREE:
      return m_renderer->get_monocular_render_state(m_activeSubwindowIndex);
    default:
      // This should never happen.
      throw std::runtime_error("Unknown camera mode");
  }
}

SubwindowConfiguration_Ptr Application::get_subwindow_configuration(size_t i) const
{
  if(m_subwindowConfigurations.size() < i + 1)
  {
    m_subwindowConfigurations.resize(i + 1);
  }

  if(!m_subwindowConfigurations[i])
  {
    m_subwindowConfigurations[i] = SubwindowConfiguration::make_default(
      i, m_pipeline->get_model()->get_slam_state(Model::get_world_scene_id())->get_depth_image_size(), m_pipeline->get_type()
    );
  }

  return m_subwindowConfigurations[i];
}

void Application::handle_key_down(const SDL_Keysym& keysym)
{
  m_inputState.press_key(static_cast<Keycode>(keysym.sym));

  // If the P key is pressed, toggle pose mirroring.
  if(keysym.sym == KEYCODE_p)
  {
    m_usePoseMirroring = !m_usePoseMirroring;
  }

  // If the semi-colon key is pressed, toggle whether or not median filtering is used when rendering the scene raycast.
  if(keysym.sym == KEYCODE_SEMICOLON)
  {
    m_renderer->set_median_filtering_enabled(!m_renderer->get_median_filtering_enabled());
  }

  // If the quote key is pressed:
  if(keysym.sym == KEYCODE_QUOTE)
  {
    // Toggle whether or not supersampling is used when rendering the scene raycast.
    m_renderer->set_supersampling_enabled(!m_renderer->get_supersampling_enabled());

    // Let the pipeline know that the raycast result size may have changed.
    const Vector2i& imgSize = get_active_subwindow().get_image()->noDims;
    m_pipeline->update_raycast_result_size(imgSize.x * imgSize.y);
  }

  // If / is pressed on its own, save a screenshot. If left shift + / is pressed, toggle sequence recording.
  // If right shift + / is pressed, toggle video recording.
  if(keysym.sym == SDLK_SLASH)
  {
    if(m_inputState.key_down(KEYCODE_LSHIFT)) toggle_recording("sequence", m_sequencePathGenerator);
    else if(m_inputState.key_down(KEYCODE_RSHIFT)) toggle_recording("video", m_videoPathGenerator);
    else save_screenshot();
  }

  // If we're running in batch mode, ignore all other keypresses.
  if(m_batchModeEnabled) return;

  // If the B key is pressed, arrange for all subsequent frames to be processed without pausing.
  if(keysym.sym == KEYCODE_b)
  {
    m_pauseBetweenFrames = false;
    m_paused = false;
  }

  // If the F key is pressed, toggle whether or not fusion is run for the active scene.
  if(keysym.sym == KEYCODE_f)
  {
    const std::string& sceneID = get_active_scene_id();
    m_pipeline->set_fusion_enabled(sceneID, !m_pipeline->get_fusion_enabled(sceneID));
  }

  // If the N key is pressed, arrange for just the next frame to be processed and enable pausing between frames.
  if(keysym.sym == KEYCODE_n)
  {
    m_pauseBetweenFrames = true;
    m_paused = false;
  }

  // If the O key is pressed, toggle segmentation output.
  if(keysym.sym == KEYCODE_o)
  {
    m_pipeline->toggle_segmentation_output();
  }

  // If left control + R is pressed, reset the active scene.
  if(keysym.sym == KEYCODE_r && m_inputState.key_down(KEYCODE_LCTRL))
  {
    m_pipeline->reset_scene(get_active_scene_id());
  }

  if(keysym.sym == KEYCODE_BACKSPACE)
  {
    const Model_Ptr& model = m_pipeline->get_model();
    const std::string& sceneID = get_active_scene_id();
    if(m_inputState.key_down(KEYCODE_RCTRL) && m_inputState.key_down(KEYCODE_RSHIFT))
    {
      // If right control + right shift + backspace is pressed, clear the semantic labels of all the voxels in the active scene, and reset the random forest and command manager.
      model->clear_labels(sceneID, ClearingSettings(CLEAR_ALL, 0, 0));
      m_pipeline->reset_forest(sceneID);
      m_commandManager.reset();
    }
    else if(m_inputState.key_down(KEYCODE_RCTRL))
    {
      // If right control + backspace is pressed, clear the labels of all voxels with the current semantic label, and reset the command manager.
      model->clear_labels(sceneID, ClearingSettings(CLEAR_EQ_LABEL, 0, model->get_semantic_label()));
      m_commandManager.reset();
    }
    else if(m_inputState.key_down(KEYCODE_RSHIFT))
    {
      // If right shift + backspace is pressed, clear the semantic labels of all the voxels in the active scene that were not labelled by the user.
      model->clear_labels(sceneID, ClearingSettings(CLEAR_NEQ_GROUP, SpaintVoxel::LG_USER, 0));
    }
    else
    {
      // If backspace is pressed on its own, clear the labels of all voxels with the current semantic label that were not labelled by the user.
      model->clear_labels(sceneID, ClearingSettings(CLEAR_EQ_LABEL_NEQ_GROUP, SpaintVoxel::LG_USER, model->get_semantic_label()));
    }
  }

  // If the H key is pressed, print out a list of controls.
  if(keysym.sym == KEYCODE_h)
  {
    std::cout << "\nKeyboard Controls:\n\n"
              << "W = Forwards\n"
              << "S = Backwards\n"
              << "A = Strafe Left\n"
              << "D = Strafe Right\n"
              << "Q = Move Up\n"
              << "E = Move Down\n"
              << "Shift + Q = Rotate Left\n"
              << "Shift + E = Rotate Right\n"
              << "F = Toggle Fusion\n"
              << "G = Set Up Vector\n"
              << "O = Toggle Segmentation Output\n"
              << "P = Toggle Pose Mirroring\n"
              << "Up = Look Down\n"
              << "Down = Look Up\n"
              << "Left = Turn Left\n"
              << "Right = Turn Right\n"
              << "C + 1 = To Semantic Lambertian Raycast\n"
              << "C + 2 = To Semantic Phong Raycast\n"
              << "C + 3 = To Semantic Colour Raycast\n"
              << "C + 4 = To Semantic Flat Raycast\n"
              << "C + 5 = To Colour Raycast\n"
              << "C + 6 = To Normal Raycast\n"
              << "C + 7 = To Depth Raycast\n"
              << "C + 8 = To Confidence Raycast\n"
              << "C + 9 = To Colour Input\n"
              << "C + 0 = To Depth Input\n"
              << "I + 1 = To Null Selector\n"
              << "I + 2 = To Picking Selector\n"
              << "I + 3 = To Leap Selector\n"
              << "I + 4 = To Touch Selector\n"
              << "L = Label With Leap Selector (Whilst Held)\n"
              << "M + 1 = To Normal Mode\n"
              << "M + 2 = To Propagation Mode\n"
              << "M + 3 = To Training Mode\n"
              << "M + 4 = To Prediction Mode\n"
              << "M + 5 = To Correction Mode\n"
              << "M + 6 = To Smoothing Mode\n"
              << "M + 7 = To Feature Inspection Mode\n"
              << "K + 1 = Disable Fiducials\n"
              << "K + 2 = Only Detect Fiducials\n"
              << "K + 3 = Only Render Fiducials\n"
              << "K + 4 = Detect and Render Fiducials\n"
              << "R + # = To Windowed Renderer (Specified Subwindow Configuration)\n"
              << "RShift + R + 1 = To Rift Renderer (Windowed)\n"
              << "RShift + R + 2 = To Rift Renderer (Fullscreen)\n"
              << "V + 1 = To Follow Camera Mode\n"
              << "V + 2 = To Free Camera Mode\n"
              << "B = Process All Frames\n"
              << "N = Process Next Frame\n"
              << "LCtrl + R = Reset Scene\n"
              << "[ = Decrease Picking Selection Radius\n"
              << "] = Increase Picking Selection Radius\n"
              << "RShift + [ = To Previous Semantic Label\n"
              << "RShift + ] = To Next Semantic Label\n"
              << "Backspace = Clear Current Label Propagation\n"
              << "RShift + Backspace = Clear All Label Propagations\n"
              << "RCtrl + Backspace = Clear Current Label\n"
              << "RCtrl + RShift + Backspace = Reset Classifier (Clear Labels and Forest)\n"
              << "; = Toggle Median Filtering\n"
              << "' = Toggle Supersampling\n"
              << "/ = Save Screenshot\n"
              << "LShift + / = Toggle Sequence Recording\n"
              << "RShift + / = Toggle Video Recording\n"
              << '\n'
              << "Joystick Controls:\n\n"
              << "Left Analog Stick = Move Camera\n"
              << "Right Analog Stick = Look Up/Down/Left/Right\n"
              << "L1 = Move Up\n"
              << "R1 = Move Down\n"
              << "L2 = Rotate Left\n"
              << "R2 = Rotate Right\n"
              << "Triangle = Set Up Vector\n";
  }
}

void Application::handle_key_up(const SDL_Keysym& keysym)
{
  m_inputState.release_key(static_cast<Keycode>(keysym.sym));
}

void Application::handle_mousebutton_down(const SDL_MouseButtonEvent& e)
{
  m_fracWindowPos = m_renderer->compute_fractional_window_position(e.x, e.y);
  SubwindowConfiguration_CPtr config = m_renderer->get_subwindow_configuration();
  boost::optional<std::pair<size_t,Vector2f> > fracSubwindowPos = config->compute_fractional_subwindow_position(m_fracWindowPos);
  if(!fracSubwindowPos) return;

  switch(e.button)
  {
    case SDL_BUTTON_LEFT:
      m_inputState.press_mouse_button(MOUSE_BUTTON_LEFT, fracSubwindowPos->second.x, fracSubwindowPos->second.y);
      break;
    case SDL_BUTTON_MIDDLE:
      m_inputState.press_mouse_button(MOUSE_BUTTON_MIDDLE, fracSubwindowPos->second.x, fracSubwindowPos->second.y);
      break;
    case SDL_BUTTON_RIGHT:
      m_inputState.press_mouse_button(MOUSE_BUTTON_RIGHT, fracSubwindowPos->second.x, fracSubwindowPos->second.y);
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
  // Allow the user to change the camera mode of the active sub-window.
  Subwindow& activeSubwindow = get_active_subwindow();
  if(m_inputState.key_down(KEYCODE_v))
  {
    if(m_inputState.key_down(KEYCODE_1)) activeSubwindow.set_camera_mode(Subwindow::CM_FOLLOW);
    else if(m_inputState.key_down(KEYCODE_2)) activeSubwindow.set_camera_mode(Subwindow::CM_FREE);
  }

  // If the active sub-window is in free camera mode, allow the user to move its camera around.
  if(activeSubwindow.get_camera_mode() == Subwindow::CM_FREE)
  {
    const float SPEED = 0.1f;
    const float ANGULAR_SPEED = 0.05f;

    MoveableCamera_Ptr camera = activeSubwindow.get_camera();

    // Set the UP vector according to the current state of the camera.
    if(m_inputState.key_down(KEYCODE_g) || m_inputState.joystick_button_down(PS3_BUTTON_TRIANGLE)) activeSubwindow.set_camera_up_vector(camera->v());

    // Get the subwindow's specific UP vector.
    const Eigen::Vector3f UP = activeSubwindow.get_camera_up_vector();

    // Handle keyboard.
    if(m_inputState.key_down(KEYCODE_w)) camera->move_n(SPEED);
    if(m_inputState.key_down(KEYCODE_s)) camera->move_n(-SPEED);
    if(m_inputState.key_down(KEYCODE_d)) camera->move_u(-SPEED);
    if(m_inputState.key_down(KEYCODE_a)) camera->move_u(SPEED);
    if(m_inputState.key_down(KEYCODE_q) && !m_inputState.key_down(KEYCODE_LSHIFT)) camera->move(UP, SPEED);
    if(m_inputState.key_down(KEYCODE_e) && !m_inputState.key_down(KEYCODE_LSHIFT)) camera->move(UP, -SPEED);

    if(m_inputState.key_down(KEYCODE_RIGHT)) camera->rotate(UP, -ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_LEFT)) camera->rotate(UP, ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_UP)) camera->rotate(camera->u(), ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_DOWN)) camera->rotate(camera->u(), -ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_q) && m_inputState.key_down(KEYCODE_LSHIFT)) camera->rotate(camera->n(), -ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_e) && m_inputState.key_down(KEYCODE_LSHIFT)) camera->rotate(camera->n(), ANGULAR_SPEED);

    // Handle joystick.
    const float JOYSTICK_THRESHOLD = 0.1f; // To avoid analog jitter.

    const float translationX = joystick_normalise_signed_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_ANALOG_LEFT_X));
    if(std::abs(translationX) > JOYSTICK_THRESHOLD) camera->move_u(-translationX * SPEED);

    const float translationY = joystick_normalise_signed_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_ANALOG_LEFT_Y));
    if(std::abs(translationY) > JOYSTICK_THRESHOLD) camera->move_n(-translationY * SPEED);

    const float rotationX = joystick_normalise_signed_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_ANALOG_RIGHT_X));
    if(std::abs(rotationX) > JOYSTICK_THRESHOLD) camera->rotate(UP, -rotationX * ANGULAR_SPEED);

    const float rotationY = joystick_normalise_signed_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_ANALOG_RIGHT_Y));
    if(std::abs(rotationY) > JOYSTICK_THRESHOLD) camera->rotate(camera->u(), rotationY * ANGULAR_SPEED);

    const float rotationZ_Left = joystick_normalise_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_TRIGGER_L2));
    if(rotationZ_Left > JOYSTICK_THRESHOLD) camera->rotate(camera->n(), -rotationZ_Left * ANGULAR_SPEED);

    const float rotationZ_Right = joystick_normalise_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_TRIGGER_R2));
    if(rotationZ_Right > JOYSTICK_THRESHOLD) camera->rotate(camera->n(), rotationZ_Right * ANGULAR_SPEED);

    const float moveUp = joystick_normalise_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_TRIGGER_L1));
    if(moveUp > JOYSTICK_THRESHOLD) camera->move(UP, moveUp * SPEED);

    const float moveDown = joystick_normalise_axis_state(m_inputState.joystick_axis_state(PS3_AXIS_TRIGGER_R1));
    if(moveDown > JOYSTICK_THRESHOLD) camera->move(UP, -moveDown * SPEED);

    // If pose mirroring is enabled, set the cameras of all other sub-windows that show the same scene
    // and are in free camera mode to have the same pose as this one.
    if(m_usePoseMirroring)
    {
      const SubwindowConfiguration_Ptr& subwindowConfiguration = m_renderer->get_subwindow_configuration();
      for(size_t i = 0, subwindowCount = subwindowConfiguration->subwindow_count(); i < subwindowCount; ++i)
      {
        Subwindow& subwindow = subwindowConfiguration->subwindow(i);
        if(subwindow.get_scene_id() == get_active_scene_id() && subwindow.get_camera_mode() == Subwindow::CM_FREE)
        {
          subwindow.get_camera()->set_from(*camera);
        }
      }
    }
  }
}

void Application::process_command_input()
{
  static bool blockUndo = false;
  if(m_inputState.key_down(KEYCODE_LCTRL) && m_inputState.key_down(KEYCODE_z))
  {
    if(!blockUndo && m_commandManager.can_undo())
    {
      m_commandManager.undo();
      blockUndo = true;
    }
  }
  else blockUndo = false;

  static bool blockRedo = false;
  if(m_inputState.key_down(KEYCODE_LCTRL) && m_inputState.key_down(KEYCODE_y))
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
      {
        m_fracWindowPos = m_renderer->compute_fractional_window_position(event.motion.x, event.motion.y);
        SubwindowConfiguration_CPtr config = m_renderer->get_subwindow_configuration();
        boost::optional<std::pair<size_t,Vector2f> > fracSubwindowPos = config->compute_fractional_subwindow_position(m_fracWindowPos);
        if(fracSubwindowPos)
        {
          m_activeSubwindowIndex = fracSubwindowPos->first;
          m_inputState.set_mouse_position(fracSubwindowPos->second.x, fracSubwindowPos->second.y);
        }
        break;
      }
      case SDL_JOYAXISMOTION:
        // We don't care from which joystick the event came from.
        m_inputState.set_joystick_axis_state(static_cast<JoystickAxis>(event.jaxis.axis), event.jaxis.value);
        break;
      case SDL_JOYBUTTONDOWN:
        m_inputState.press_joystick_button(static_cast<JoystickButton>(event.jbutton.button));
        break;
      case SDL_JOYBUTTONUP:
        m_inputState.release_joystick_button(static_cast<JoystickButton>(event.jbutton.button));
        break;
      case SDL_QUIT:
        return false;
      default:
        break;
    }
  }

  return true;
}

void Application::process_fiducial_input()
{
  if(m_inputState.key_down(KEYCODE_k))
  {
    bool k1 = m_inputState.key_down(KEYCODE_1),
         k2 = m_inputState.key_down(KEYCODE_2),
         k3 = m_inputState.key_down(KEYCODE_3),
         k4 = m_inputState.key_down(KEYCODE_4);

    if(k1 || k2 || k3 || k4)
    {
      const std::string& sceneID = get_active_scene_id();
      m_pipeline->set_detect_fiducials(sceneID, k2 || k4);
      m_renderFiducials = k3 || k4;
    }
  }
}

void Application::process_input()
{
  process_camera_input();
  process_renderer_input();

  // If we are running in batch mode, suppress all non-essential input.
  if(m_batchModeEnabled) return;

  process_command_input();
  process_fiducial_input();
  process_labelling_input();
  process_mode_input();
  process_voice_input();
}

void Application::process_labelling_input()
{
  // Allow the user to change the current semantic label.
  static bool canChangeLabel = true;
  const Model_Ptr& model = m_pipeline->get_model();
  LabelManager_CPtr labelManager = model->get_label_manager();
  SpaintVoxel::Label semanticLabel = model->get_semantic_label();

  if(m_inputState.key_down(KEYCODE_RSHIFT) && m_inputState.key_down(KEYCODE_RIGHTBRACKET))
  {
    if(canChangeLabel) semanticLabel = labelManager->get_next_label(semanticLabel);
    canChangeLabel = false;
  }
  else if(m_inputState.key_down(KEYCODE_RSHIFT) && m_inputState.key_down(KEYCODE_LEFTBRACKET))
  {
    if(canChangeLabel) semanticLabel = labelManager->get_previous_label(semanticLabel);
    canChangeLabel = false;
  }
  else canChangeLabel = true;

  model->set_semantic_label(semanticLabel);

  // Update the current selector.
  model->update_selector(m_inputState, model->get_slam_state(get_active_scene_id()), get_monocular_render_state(), m_renderer->is_mono());

  // Record whether or not we're in the middle of marking some voxels (this allows us to make voxel marking atomic for undo/redo purposes).
  static bool currentlyMarking = false;

  // Specify the precursors map for compressible commands.
  static const std::string beginMarkVoxelsDesc = "Begin Mark Voxels";
  static const std::string markVoxelsDesc = MarkVoxelsCommand::get_static_description();
  static std::map<std::string,std::string> precursors = map_list_of(beginMarkVoxelsDesc,markVoxelsDesc)(markVoxelsDesc,markVoxelsDesc);

  // If the current selector is active:
  if(model->get_selector()->is_active())
  {
    // Get the voxels selected by the user (if any).
    Selector::Selection_CPtr selection = model->get_selection();

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
        m_commandManager.execute_compressible_command(Command_CPtr(new MarkVoxelsCommand(get_active_scene_id(), selection, packedLabel, model)), precursors);
      }
      else model->mark_voxels(get_active_scene_id(), selection, packedLabel, NORMAL_MARKING);
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
  MultiScenePipeline::Mode mode = m_pipeline->get_mode();
  if(m_inputState.key_down(KEYCODE_m))
  {
    if(m_inputState.key_down(KEYCODE_1))      mode = MultiScenePipeline::MODE_NORMAL;
    else if(m_inputState.key_down(KEYCODE_2)) mode = MultiScenePipeline::MODE_PROPAGATION;
    else if(m_inputState.key_down(KEYCODE_3)) mode = MultiScenePipeline::MODE_TRAINING;
    else if(m_inputState.key_down(KEYCODE_4)) mode = MultiScenePipeline::MODE_PREDICTION;
    else if(m_inputState.key_down(KEYCODE_5)) mode = MultiScenePipeline::MODE_TRAIN_AND_PREDICT;
    else if(m_inputState.key_down(KEYCODE_6)) mode = MultiScenePipeline::MODE_SMOOTHING;
    else if(m_inputState.key_down(KEYCODE_7)) mode = MultiScenePipeline::MODE_FEATURE_INSPECTION;
    else if(m_inputState.key_down(KEYCODE_8)) mode = MultiScenePipeline::MODE_SEGMENTATION_TRAINING;
    else if(m_inputState.key_down(KEYCODE_9)) mode = MultiScenePipeline::MODE_SEGMENTATION;
  }
  m_pipeline->set_mode(mode);
}

void Application::process_renderer_input()
{
  // Allow the user to switch renderers.
  static int framesTillSwitchAllowed = 0;
  const int SWITCH_DELAY = 20;
  if(framesTillSwitchAllowed == 0)
  {
    if(m_inputState.key_down(KEYCODE_r))
    {
      if(m_inputState.key_down(KEYCODE_LSHIFT))
      {
#ifdef WITH_OVR
        if(m_inputState.key_down(KEYCODE_1) || m_inputState.key_down(KEYCODE_2))
        {
          try
          {
            switch_to_rift_renderer(m_inputState.key_down(KEYCODE_1) ? RiftRenderer::WINDOWED_MODE : RiftRenderer::FULLSCREEN_MODE);
            framesTillSwitchAllowed = SWITCH_DELAY;
          }
          catch(std::runtime_error& e)
          {
            std::cerr << e.what() << '\n';
          }
        }
#endif
      }
      else
      {
        for(size_t subwindowConfigurationIndex = 0; subwindowConfigurationIndex <= 9; ++subwindowConfigurationIndex)
        {
          if(m_inputState.key_down(static_cast<Keycode>(KEYCODE_0 + subwindowConfigurationIndex)))
          {
            switch_to_windowed_renderer(subwindowConfigurationIndex);
            framesTillSwitchAllowed = SWITCH_DELAY;
            break;
          }
        }
      }
    }
  }
  else --framesTillSwitchAllowed;

  // Allow the user to change the visualisation type of the active sub-window.
  if(m_inputState.key_down(KEYCODE_c))
  {
    Subwindow& subwindow = get_active_subwindow();
    boost::optional<VisualisationGenerator::VisualisationType> type =
      m_inputState.key_down(KEYCODE_1) ? VisualisationGenerator::VT_SCENE_SEMANTICLAMBERTIAN :
      m_inputState.key_down(KEYCODE_2) ? VisualisationGenerator::VT_SCENE_SEMANTICPHONG :
      m_inputState.key_down(KEYCODE_3) ? VisualisationGenerator::VT_SCENE_SEMANTICCOLOUR :
      m_inputState.key_down(KEYCODE_4) ? VisualisationGenerator::VT_SCENE_SEMANTICFLAT :
      m_inputState.key_down(KEYCODE_5) ? VisualisationGenerator::VT_SCENE_COLOUR :
      m_inputState.key_down(KEYCODE_6) ? VisualisationGenerator::VT_SCENE_NORMAL :
      m_inputState.key_down(KEYCODE_7) ? VisualisationGenerator::VT_SCENE_DEPTH :
      m_inputState.key_down(KEYCODE_8) ? VisualisationGenerator::VT_SCENE_CONFIDENCE :
      m_inputState.key_down(KEYCODE_9) ? VisualisationGenerator::VT_INPUT_COLOUR :
      m_inputState.key_down(KEYCODE_0) ? boost::optional<VisualisationGenerator::VisualisationType>(VisualisationGenerator::VT_INPUT_DEPTH) :
      boost::none;

    if(type)
    {
      subwindow.set_type(*type);
      subwindow.set_surfel_flag(m_inputState.key_down(KEYCODE_LSHIFT));
    }
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
    const LabelManager_Ptr& labelManager = m_pipeline->get_model()->get_label_manager();
    for(size_t i = 0, labelCount = labelManager->get_label_count(); i < labelCount; ++i)
    {
      SpaintVoxel::Label label = static_cast<SpaintVoxel::Label>(i);
      std::string changeLabelCommand = "label " + labelManager->get_label_name(label);
      if(command == changeLabelCommand) m_pipeline->get_model()->set_semantic_label(label);
    }

    // Process any requests to disable/enable fusion for the active scene.
    if(command == "disable fusion") m_pipeline->set_fusion_enabled(get_active_scene_id(), false);
    if(command == "enable fusion") m_pipeline->set_fusion_enabled(get_active_scene_id(), true);

    // Process any requests to change pipeline mode.
    if(command == "switch to normal mode") m_pipeline->set_mode(MultiScenePipeline::MODE_NORMAL);
    if(command == "switch to propagation mode") m_pipeline->set_mode(MultiScenePipeline::MODE_PROPAGATION);
    if(command == "switch to training mode") m_pipeline->set_mode(MultiScenePipeline::MODE_TRAINING);
    if(command == "switch to prediction mode") m_pipeline->set_mode(MultiScenePipeline::MODE_PREDICTION);
    if(command == "switch to correction mode") m_pipeline->set_mode(MultiScenePipeline::MODE_TRAIN_AND_PREDICT);
    if(command == "switch to smoothing mode") m_pipeline->set_mode(MultiScenePipeline::MODE_SMOOTHING);
  }
}

void Application::save_mesh() const
{
  if(!m_meshingEngine) return;

  Model_CPtr model = m_pipeline->get_model();
  const Settings_CPtr& settings = model->get_settings();

  const Subwindow& mainSubwindow = m_renderer->get_subwindow_configuration()->subwindow(0);
  const std::string& sceneID = mainSubwindow.get_scene_id();
  SpaintVoxelScene_CPtr scene = model->get_slam_state(sceneID)->get_voxel_scene();

  // Construct the mesh.
  Mesh_Ptr mesh(new ITMMesh(settings->GetMemoryType()));
  m_meshingEngine->MeshScene(mesh.get(), scene.get());

  // Find the meshes directory and make sure that it exists.
  boost::filesystem::path meshesSubdir = find_subdir_from_executable("meshes");
  boost::filesystem::create_directories(meshesSubdir);

  // Determine the filename to use for the mesh, based on either the experiment tag (if specified) or the current timestamp (otherwise).
  const std::string meshFilename = settings->get_first_value<std::string>("experimentTag", "spaint-" + TimeUtil::get_iso_timestamp()) + ".stl";
  const boost::filesystem::path meshPath = meshesSubdir / meshFilename;

  // Save the mesh to disk.
  std::cout << "Saving mesh to: " << meshPath << '\n';
  mesh->WriteSTL(meshPath.string().c_str());
}

void Application::save_screenshot() const
{
  boost::filesystem::path p = find_subdir_from_executable("screenshots") / ("spaint-" + TimeUtil::get_iso_timestamp() + ".png");
  boost::filesystem::create_directories(p.parent_path());
  std::cout << "[spaint] Saving screenshot to " << p << "...\n";
  ImagePersister::save_image_on_thread(m_renderer->capture_screenshot(), p);
}

void Application::save_sequence_frame()
{
  const Subwindow& mainSubwindow = m_renderer->get_subwindow_configuration()->subwindow(0);
  const std::string& sceneID = mainSubwindow.get_scene_id();

  // If the RGBD calibration hasn't already been saved, save it now.
  SLAMState_CPtr slamState = m_pipeline->get_model()->get_slam_state(sceneID);
  boost::filesystem::path calibrationFile = m_sequencePathGenerator->get_base_dir() / "calib.txt";
  if(!boost::filesystem::exists(calibrationFile))
  {
    writeRGBDCalib(calibrationFile.string().c_str(), slamState->get_view()->calib);
  }

  // Save the current input images.
  ImagePersister::save_image_on_thread(slamState->get_input_raw_depth_image_copy(), m_sequencePathGenerator->make_path("depthm%06i.pgm"));
  ImagePersister::save_image_on_thread(slamState->get_input_rgb_image_copy(), m_sequencePathGenerator->make_path("rgbm%06i.ppm"));

  // Save the inverse pose (i.e. the camera -> world transformation).
  PosePersister::save_pose_on_thread(slamState->get_pose().GetInvM(), m_sequencePathGenerator->make_path("posem%06i.txt"));

  m_sequencePathGenerator->increment_index();
}

void Application::save_video_frame()
{
  m_videoPathGenerator->increment_index();
  ImagePersister::save_image_on_thread(m_renderer->capture_screenshot(), m_videoPathGenerator->make_path("%06i.png"));
}

void Application::setup_labels()
{
  const LabelManager_Ptr& labelManager = m_pipeline->get_model()->get_label_manager();
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
  m_pipeline->get_model()->set_semantic_label(1);
}

void Application::setup_meshing()
{
  const Settings_CPtr& settings = m_pipeline->get_model()->get_settings();
  if(settings->createMeshingEngine || m_saveMeshOnExit)
  {
    m_meshingEngine.reset(ITMMeshingEngineFactory::MakeMeshingEngine<SpaintVoxel,ITMVoxelBlockHash>(settings->deviceType));
  }
}

#ifdef WITH_OVR
void Application::switch_to_rift_renderer(RiftRenderer::RiftRenderingMode mode)
{
  const size_t riftSubwindowConfigurationIndex = 1;
  SubwindowConfiguration_Ptr subwindowConfiguration = get_subwindow_configuration(riftSubwindowConfigurationIndex);
  if(!subwindowConfiguration) return;

  m_renderer.reset(new RiftRenderer("Semantic Paint", m_pipeline->get_model(), subwindowConfiguration, mode));
}
#endif

void Application::switch_to_windowed_renderer(size_t subwindowConfigurationIndex)
{
  SubwindowConfiguration_Ptr subwindowConfiguration = get_subwindow_configuration(subwindowConfigurationIndex);
  if(!subwindowConfiguration) return;

  const Subwindow& mainSubwindow = subwindowConfiguration->subwindow(0);
  const Vector2i& mainImageSize = m_pipeline->get_model()->get_slam_state(Model::get_world_scene_id())->get_depth_image_size();
  Vector2i windowViewportSize((int)ROUND(mainImageSize.width / mainSubwindow.width()), (int)ROUND(mainImageSize.height / mainSubwindow.height()));

  m_renderer.reset(new WindowedRenderer("Semantic Paint", m_pipeline->get_model(), subwindowConfiguration, windowViewportSize));
}

void Application::toggle_recording(const std::string& type, boost::optional<tvgutil::SequentialPathGenerator>& pathGenerator)
{
  if(pathGenerator)
  {
    pathGenerator.reset();
    std::cout << "[spaint] Stopped saving " << type << ".\n";
  }
  else
  {
    pathGenerator.reset(SequentialPathGenerator(find_subdir_from_executable(type + "s") / TimeUtil::get_iso_timestamp()));
    boost::filesystem::create_directories(pathGenerator->get_base_dir());
    std::cout << "[spaint] Started saving " << type << " to " << pathGenerator->get_base_dir() << "...\n";
  }
}
