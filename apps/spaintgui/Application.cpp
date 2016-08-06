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

#include <ITMLib/Objects/Camera/ITMCalibIO.h>
using namespace ITMLib;

#include <rigging/MoveableCamera.h>
using namespace rigging;

#include <spaint/ogl/WrappedGL.h>
#include <spaint/util/ImagePersister.h>
using namespace spaint;

#include <tvgutil/PathFinder.h>
#include <tvgutil/commands/NoOpCommand.h>
#include <tvgutil/timing/TimeUtil.h>
using namespace tvgutil;

#ifdef WITH_OVR
#include "renderers/RiftRenderer.h"
#endif
#include "renderers/WindowedRenderer.h"

#include "commands/MarkVoxelsCommand.h"

//#################### CONSTRUCTORS ####################

Application::Application(const Pipeline_Ptr& pipeline)
: m_activeSubwindowIndex(0),
  m_commandManager(10),
  m_pauseBetweenFrames(true),
  m_paused(true),
  m_pipeline(pipeline),
  m_voiceCommandStream("localhost", "23984")
{
  setup_labels();
  switch_to_windowed_renderer(1);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Application::run()
{
  for(;;)
  {
    if(!process_events() || m_inputState.key_down(KEYCODE_ESCAPE)) return;

    // Take action as relevant based on the current input state.
    process_input();

    // If the application is unpaused, process a new frame.
    if(!m_paused)
    {
      // Run the main section of the pipeline.
      bool frameWasProcessed = m_pipeline->run_main_section();

      // If a new frame was processed and we're currently recording the sequence, save the frame to disk.
      if(frameWasProcessed && m_sequencePathGenerator)
      {
        save_sequence_frame();
      }
    }

    // Render the scene.
    m_renderer->render(m_pipeline->get_interactor(), m_fracWindowPos);

    // If the application is unpaused, run the mode-specific section of the pipeline.
    if(!m_paused) m_pipeline->run_mode_specific_section(get_monocular_render_state());

    // If we're currently recording a video, save the next frame of it to disk.
    if(m_videoPathGenerator) save_video_frame();

    // If desired, pause at the end of each frame for debugging purposes.
    if(m_pauseBetweenFrames) m_paused = true;
  }
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::filesystem::path Application::resources_dir()
{
  return find_subdir_from_executable("resources");
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

Application::RenderState_CPtr Application::get_monocular_render_state() const
{
  switch(m_renderer->get_camera_mode())
  {
    case Renderer::CM_FOLLOW:
      return m_pipeline->get_raycaster()->get_live_render_state();
    case Renderer::CM_FREE:
      return m_renderer->get_monocular_render_state();
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
    m_subwindowConfigurations[i] = SubwindowConfiguration::make_default(i, m_pipeline->get_model()->get_depth_image_size());
  }

  return m_subwindowConfigurations[i];
}

void Application::handle_key_down(const SDL_Keysym& keysym)
{
  m_inputState.press_key(static_cast<Keycode>(keysym.sym));

  // If the B key is pressed, arrange for all subsequent frames to be processed without pausing.
  if(keysym.sym == KEYCODE_b)
  {
    m_pauseBetweenFrames = false;
    m_paused = false;
  }

  // If the F key is pressed, toggle whether or not fusion is run as part of the pipeline.
  if(keysym.sym == KEYCODE_f)
  {
    m_pipeline->set_fusion_enabled(!m_pipeline->get_fusion_enabled());
  }

  // If the N key is pressed, arrange for just the next frame to be processed and enable pausing between frames.
  if(keysym.sym == KEYCODE_n)
  {
    m_pauseBetweenFrames = true;
    m_paused = false;
  }

  if(keysym.sym == KEYCODE_BACKSPACE)
  {
    const Interactor_Ptr& interactor = m_pipeline->get_interactor();
    if(m_inputState.key_down(KEYCODE_RCTRL) && m_inputState.key_down(KEYCODE_RSHIFT))
    {
      // If right control + right shift + backspace is pressed, clear the semantic labels of all the voxels in the scene, and reset the random forest and command manager.
      interactor->clear_labels(ClearingSettings(CLEAR_ALL, 0, 0));
      m_pipeline->reset_forest();
      m_commandManager.reset();
    }
    else if(m_inputState.key_down(KEYCODE_RCTRL))
    {
      // If right control + backspace is pressed, clear the labels of all voxels with the current semantic label, and reset the command manager.
      interactor->clear_labels(ClearingSettings(CLEAR_EQ_LABEL, 0, interactor->get_semantic_label()));
      m_commandManager.reset();
    }
    else if(m_inputState.key_down(KEYCODE_RSHIFT))
    {
      // If right shift + backspace is pressed, clear the semantic labels of all the voxels in the scene that were not labelled by the user.
      interactor->clear_labels(ClearingSettings(CLEAR_NEQ_GROUP, SpaintVoxel::LG_USER, 0));
    }
    else
    {
      // If backspace is pressed on its own, clear the labels of all voxels with the current semantic label that were not labelled by the user.
      interactor->clear_labels(ClearingSettings(CLEAR_EQ_LABEL_NEQ_GROUP, SpaintVoxel::LG_USER, interactor->get_semantic_label()));
    }
  }

  // If the semi-colon key is pressed, toggle whether or not median filtering is used when rendering the scene raycast.
  if(keysym.sym == KEYCODE_SEMICOLON)
  {
    m_renderer->set_median_filtering_enabled(!m_renderer->get_median_filtering_enabled());
  }

  // If / is pressed on its own, save a screenshot. If left shift + / is pressed, toggle sequence recording.
  // If right shift + / is pressed, toggle video recording.
  if(keysym.sym == SDLK_SLASH)
  {
    if(m_inputState.key_down(KEYCODE_LSHIFT)) toggle_recording("sequence", m_sequencePathGenerator);
    else if(m_inputState.key_down(KEYCODE_RSHIFT)) toggle_recording("video", m_videoPathGenerator);
    else save_screenshot();
  }

  // If the H key is pressed, print out a list of keyboard controls.
  if(keysym.sym == KEYCODE_h)
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
              << "M + 2 = To Propagation Mode\n"
              << "M + 3 = To Training Mode\n"
              << "M + 4 = To Prediction Mode\n"
              << "M + 5 = To Correction Mode\n"
              << "M + 6 = To Smoothing Mode\n"
              << "M + 7 = To Feature Inspection Mode\n"
              << "R + 1 = To Windowed Renderer\n"
              << "R + 2 = To Rift Renderer (Windowed)\n"
              << "R + 3 = To Rift Renderer (Fullscreen)\n"
              << "V + 1 = To Follow Camera Mode\n"
              << "V + 2 = To Free Camera Mode\n"
              << "B = Process All Frames\n"
              << "N = Process Next Frame\n"
              << "[ = Decrease Picking Selection Radius\n"
              << "] = Increase Picking Selection Radius\n"
              << "RShift + [ = To Previous Semantic Label\n"
              << "RShift + ] = To Next Semantic Label\n"
              << "Backspace = Clear Current Label Propagation\n"
              << "RShift + Backspace = Clear All Label Propagations\n"
              << "RCtrl + Backspace = Clear Current Label\n"
              << "RCtrl + RShift + Backspace = Reset (Clear Labels and Forest)\n"
              << "; = Toggle Median Filtering\n"
              << "/ = Save Screenshot\n"
              << "LShift + / = Toggle Sequence Recording\n"
              << "RShift + / = Toggle Video Recording\n";
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
  // Allow the user to switch camera modes.
  if(m_inputState.key_down(KEYCODE_v))
  {
    if(m_inputState.key_down(KEYCODE_1)) m_renderer->set_camera_mode(Renderer::CM_FOLLOW);
    else if(m_inputState.key_down(KEYCODE_2)) m_renderer->set_camera_mode(Renderer::CM_FREE);
  }

  // If we're in free camera mode, allow the user to move the camera around.
  if(m_renderer->get_camera_mode() == Renderer::CM_FREE)
  {
    const float SPEED = 0.1f;
    const float ANGULAR_SPEED = 0.05f;
    static const Eigen::Vector3f UP(0.0f, -1.0f, 0.0f);

    MoveableCamera_Ptr camera = m_renderer->get_camera();

    if(m_inputState.key_down(KEYCODE_w)) camera->move_n(SPEED);
    if(m_inputState.key_down(KEYCODE_s)) camera->move_n(-SPEED);
    if(m_inputState.key_down(KEYCODE_d)) camera->move_u(-SPEED);
    if(m_inputState.key_down(KEYCODE_a)) camera->move_u(SPEED);
    if(m_inputState.key_down(KEYCODE_q)) camera->move(UP, SPEED);
    if(m_inputState.key_down(KEYCODE_e)) camera->move(UP, -SPEED);

    if(m_inputState.key_down(KEYCODE_RIGHT)) camera->rotate(UP, -ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_LEFT)) camera->rotate(UP, ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_UP)) camera->rotate(camera->u(), ANGULAR_SPEED);
    if(m_inputState.key_down(KEYCODE_DOWN)) camera->rotate(camera->u(), -ANGULAR_SPEED);
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
  const Interactor_Ptr& interactor = m_pipeline->get_interactor();
  LabelManager_CPtr labelManager = m_pipeline->get_model()->get_label_manager();
  SpaintVoxel::Label semanticLabel = interactor->get_semantic_label();

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
  Pipeline::Mode mode = m_pipeline->get_mode();
  if(m_inputState.key_down(KEYCODE_m))
  {
    if(m_inputState.key_down(KEYCODE_1))      mode = Pipeline::MODE_NORMAL;
    else if(m_inputState.key_down(KEYCODE_2)) mode = Pipeline::MODE_PROPAGATION;
    else if(m_inputState.key_down(KEYCODE_3)) mode = Pipeline::MODE_TRAINING;
    else if(m_inputState.key_down(KEYCODE_4)) mode = Pipeline::MODE_PREDICTION;
    else if(m_inputState.key_down(KEYCODE_5)) mode = Pipeline::MODE_TRAIN_AND_PREDICT;
    else if(m_inputState.key_down(KEYCODE_6)) mode = Pipeline::MODE_SMOOTHING;
    else if(m_inputState.key_down(KEYCODE_7)) mode = Pipeline::MODE_FEATURE_INSPECTION;
    else if(m_inputState.key_down(KEYCODE_8)) mode = Pipeline::MODE_SEGMENTATION_TRAINING;
    else if(m_inputState.key_down(KEYCODE_9)) mode = Pipeline::MODE_SEGMENTATION;
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
    Subwindow& subwindow = m_renderer->get_subwindow_configuration()->subwindow(m_activeSubwindowIndex);
    subwindow.set_type(
      m_inputState.key_down(KEYCODE_1) ? Raycaster::RT_SEMANTICLAMBERTIAN :
      m_inputState.key_down(KEYCODE_2) ? Raycaster::RT_SEMANTICPHONG :
      m_inputState.key_down(KEYCODE_3) ? Raycaster::RT_SEMANTICCOLOUR :
      m_inputState.key_down(KEYCODE_4) ? Raycaster::RT_SEMANTICFLAT :
      subwindow.get_type()
    );
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
      if(command == changeLabelCommand) m_pipeline->get_interactor()->set_semantic_label(label);
    }

    // Process any requests to disable/enable fusion.
    if(command == "disable fusion") m_pipeline->set_fusion_enabled(false);
    if(command == "enable fusion") m_pipeline->set_fusion_enabled(true);

    // Process any requests to change pipeline mode.
    if(command == "switch to normal mode") m_pipeline->set_mode(Pipeline::MODE_NORMAL);
    if(command == "switch to propagation mode") m_pipeline->set_mode(Pipeline::MODE_PROPAGATION);
    if(command == "switch to training mode") m_pipeline->set_mode(Pipeline::MODE_TRAINING);
    if(command == "switch to prediction mode") m_pipeline->set_mode(Pipeline::MODE_PREDICTION);
    if(command == "switch to correction mode") m_pipeline->set_mode(Pipeline::MODE_TRAIN_AND_PREDICT);
    if(command == "switch to smoothing mode") m_pipeline->set_mode(Pipeline::MODE_SMOOTHING);
  }
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
  // If the RGBD calibration hasn't already been saved, save it now.
  boost::filesystem::path calibrationFile = m_sequencePathGenerator->get_base_dir() / "calib.txt";
  if(!boost::filesystem::exists(calibrationFile))
  {
    writeRGBDCalib(calibrationFile.string().c_str(), *m_pipeline->get_model()->get_view()->calib);
  }

  // Save the current input images.
  ImagePersister::save_image_on_thread(m_pipeline->get_input_raw_depth_image_copy(), m_sequencePathGenerator->make_path("depthm%06i.pgm"));
  ImagePersister::save_image_on_thread(m_pipeline->get_input_rgb_image_copy(), m_sequencePathGenerator->make_path("rgbm%06i.ppm"));
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
  m_pipeline->get_interactor()->set_semantic_label(1);
}

#ifdef WITH_OVR
void Application::switch_to_rift_renderer(RiftRenderer::RiftRenderingMode mode)
{
  const size_t riftSubwindowConfigurationIndex = 1;
  SubwindowConfiguration_Ptr subwindowConfiguration = get_subwindow_configuration(riftSubwindowConfigurationIndex);
  if(!subwindowConfiguration) return;

  m_renderer.reset(new RiftRenderer("Semantic Paint", m_pipeline->get_model(), m_pipeline->get_raycaster(), subwindowConfiguration, mode));
}
#endif

void Application::switch_to_windowed_renderer(size_t subwindowConfigurationIndex)
{
  SubwindowConfiguration_Ptr subwindowConfiguration = get_subwindow_configuration(subwindowConfigurationIndex);
  if(!subwindowConfiguration) return;

  const Subwindow& mainSubwindow = subwindowConfiguration->subwindow(0);
  const Vector2i& depthImageSize = m_pipeline->get_model()->get_depth_image_size();
  Vector2i windowViewportSize((int)ROUND(depthImageSize.width / mainSubwindow.width()), (int)ROUND(depthImageSize.height / mainSubwindow.height()));

  m_renderer.reset(new WindowedRenderer("Semantic Paint", m_pipeline->get_model(), m_pipeline->get_raycaster(), subwindowConfiguration, windowViewportSize));
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
