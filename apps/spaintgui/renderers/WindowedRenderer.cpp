/**
 * spaintgui: WindowedRenderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "WindowedRenderer.h"
using namespace ITMLib;
using namespace ORUtils;

#include <stdexcept>

#include <rigging/SimpleCamera.h>
using namespace rigging;

#include <spaint/util/CameraPoseConverter.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

WindowedRenderer::WindowedRenderer(const std::string& title, const Model_CPtr& model, const Raycaster_CPtr& raycaster, const SubwindowConfiguration_Ptr& subwindowConfiguration,
                                   const boost::optional<Vector2i>& viewportSize)
: Renderer(model, raycaster, subwindowConfiguration, viewportSize)
{
  // Create the window into which to render.
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

  set_window(SDL_Window_Ptr(
    SDL_CreateWindow(
      title.c_str(),
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      get_viewport_size().width,
      get_viewport_size().height,
      SDL_WINDOW_OPENGL
    ),
    &SDL_DestroyWindow
  ));

  // Set up the camera.
  m_camera.reset(new SimpleCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));

  // Initialise the temporary image and texture used for visualising the scene.
  initialise_common();
}

//#################### DESTRUCTOR ####################

WindowedRenderer::~WindowedRenderer()
{
  destroy_common();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

rigging::MoveableCamera_Ptr WindowedRenderer::get_camera()
{
  return m_camera;
}

WindowedRenderer::RenderState_CPtr WindowedRenderer::get_monocular_render_state() const
{
  return m_renderState;
}

bool WindowedRenderer::is_mono() const
{
  return true;
}

void WindowedRenderer::render(const Interactor_CPtr& interactor) const
{
  // Reacquire the focus for this window if it has been lost to debugging windows.
  SDL_RaiseWindow(get_window());

  // Determine the camera pose.
  SE3Pose pose;
  switch(get_camera_mode())
  {
    case CM_FOLLOW:
      pose = get_model()->get_pose();
      break;
    case CM_FREE:
      pose = CameraPoseConverter::camera_to_pose(*m_camera);
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Error: Unknown camera mode");
  }

  // Render the scene from that camera pose.
  render_scene(pose, interactor, m_renderState);

  // Swap the front and back buffers.
  SDL_GL_SwapWindow(get_window());
}
