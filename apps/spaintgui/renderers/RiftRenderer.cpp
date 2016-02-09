/**
 * spaintgui: RiftRenderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "RiftRenderer.h"
using namespace ITMLib;
using namespace ORUtils;

#include <iostream>
#include <stdexcept>

#ifdef __linux__
// Note: This header can't be included in WrappedGL.h because it causes a conflict with Eigen/Core.
#include <GL/glx.h>
#endif

#ifdef __APPLE__
// Disable an uninteresting warning.
#pragma GCC diagnostic ignored "-Wextern-c-compat"

// Make OVR work in the absence of C++11 support.
#define static_assert(x, y)
using std::isnan;
#endif

#include <OVR_CAPI_GL.h>
#include <Extras/OVR_Math.h>

#ifdef __APPLE__
#undef static_assert
#endif

#include <SDL_syswm.h>

#include <rigging/DerivedCamera.h>
#include <rigging/SimpleCamera.h>
using namespace rigging;

#include <spaint/util/CameraPoseConverter.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

RiftRenderer::RiftRenderer(const std::string& title, const Model_CPtr& model, const Raycaster_CPtr& raycaster,
                           const SubwindowConfiguration_Ptr& subwindowConfiguration, RiftRenderingMode renderingMode)
: Renderer(model, raycaster, subwindowConfiguration, model->get_depth_image_size())
{
  // Get a handle to the Rift.
  m_hmd = ovrHmd_Create(0);
  if(!m_hmd)
  {
    std::cout << "[spaint] Could not find the Rift, attempting to fall back to a virtual Rift\n";
    m_hmd = ovrHmd_CreateDebug(ovrHmd_DK2);
    if(!m_hmd) throw std::runtime_error("[spaint] Failed to create a virtual Rift!");
  }

  // Create the window into which to render.
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  set_window(SDL_Window_Ptr(
    SDL_CreateWindow(
      title.c_str(),
      renderingMode == WINDOWED_MODE ? SDL_WINDOWPOS_UNDEFINED : SDL_WINDOWPOS_CENTERED_DISPLAY(1),
      renderingMode == WINDOWED_MODE ? SDL_WINDOWPOS_UNDEFINED : SDL_WINDOWPOS_CENTERED_DISPLAY(1),
      m_hmd->Resolution.w,
      m_hmd->Resolution.h,
      renderingMode == WINDOWED_MODE ? SDL_WINDOW_OPENGL : SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN_DESKTOP
    ),
    &SDL_DestroyWindow
  ));

  // Get device-dependent information about the window.
  SDL_SysWMinfo wmInfo;
  SDL_VERSION(&wmInfo.version);
  SDL_GetWindowWMInfo(get_window(), &wmInfo);

  // Configure rendering via the Rift SDK.
  const int backBufferMultisample = 1;
  ovrGLConfig cfg;
  cfg.OGL.Header.API = ovrRenderAPI_OpenGL;
  cfg.OGL.Header.BackBufferSize = OVR::Sizei(m_hmd->Resolution.w, m_hmd->Resolution.h);
  cfg.OGL.Header.Multisample = backBufferMultisample;
#if defined(_WIN32)
  cfg.OGL.Window = wmInfo.info.win.window;
  cfg.OGL.DC = NULL;
#elif defined(__linux__)
  cfg.OGL.Disp = glXGetCurrentDisplay();
#endif

  const unsigned int distortionCaps = ovrDistortionCap_Vignette | ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive;
  ovrEyeRenderDesc eyeRenderDesc[2];
  if(!ovrHmd_ConfigureRendering(m_hmd, &cfg.Config, distortionCaps, m_hmd->DefaultEyeFov, eyeRenderDesc))
  {
    throw std::runtime_error("[spaint] Could not configure rendering on the Rift!");
  }

#ifdef _WIN32
  // Attach the HMD to the window.
  if(!ovrHmd_AttachToWindow(m_hmd, cfg.OGL.Window, NULL, NULL))
  {
    throw std::runtime_error("[spaint] Could not attach the Rift to the window!");
  }
#endif

  // Set up the stereo camera.
  const float HALF_IPD = 0.032f; // the average (male) interpupillary distance (IPD) is about 6.4cm
  const float N_OFFSET = 0.3f;   // the amount by which we move the camera away from the scene to ensure that the user can see enough
  m_camera.reset(new CompositeCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));
  m_camera->add_secondary_camera("left", Camera_CPtr(new DerivedCamera(m_camera, Eigen::Matrix3f::Identity(), Eigen::Vector3f(HALF_IPD, 0.0f, -N_OFFSET))));
  m_camera->add_secondary_camera("right", Camera_CPtr(new DerivedCamera(m_camera, Eigen::Matrix3f::Identity(), Eigen::Vector3f(-HALF_IPD, 0.0f, -N_OFFSET))));

  // Set up the eye frame buffers.
  ORUtils::Vector2<int> depthImageSize = get_model()->get_depth_image_size();
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    m_eyeFrameBuffers[i].reset(new FrameBuffer(depthImageSize.width, depthImageSize.height));
  }

  // Initialise the temporary image and texture used for visualising the scene.
  initialise_common();
}

//#################### DESTRUCTOR ####################

RiftRenderer::~RiftRenderer()
{
  destroy_common();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

MoveableCamera_Ptr RiftRenderer::get_camera()
{
  return m_camera;
}

RiftRenderer::RenderState_CPtr RiftRenderer::get_monocular_render_state() const
{
  // The Rift is a stereo display, so return the render state corresponding to the left eye.
  return m_renderStates[ovrEye_Left];
}

bool RiftRenderer::is_mono() const
{
  return false;
}

void RiftRenderer::render(const Interactor_CPtr& interactor, const Vector2f& fracWindowPos) const
{
  // Keep trying to get rid of the annoying health and safety warning until it goes away.
  ovrHmd_DismissHSWDisplay(m_hmd);

  // Start the frame.
  ovrHmd_BeginFrame(m_hmd, 0);

  // If we're following the reconstruction, update the position and orientation of the camera.
  if(get_camera_mode() == CM_FOLLOW)
  {
    m_camera->set_from(CameraPoseConverter::pose_to_camera(get_model()->get_pose()));
  }

  // Calculate the left and right eye poses.
  SE3Pose poses[] =
  {
    CameraPoseConverter::camera_to_pose(*m_camera->get_secondary_camera("left")),
    CameraPoseConverter::camera_to_pose(*m_camera->get_secondary_camera("right"))
  };

  // Render the scene into OpenGL textures from the left and right eye poses.
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    glBindFramebuffer(GL_FRAMEBUFFER, m_eyeFrameBuffers[i]->get_id());
    glUseProgram(0);

    render_scene(poses[i], interactor, m_renderStates[i], fracWindowPos);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  // Set up the Rift eye poses and pass the eye textures to the Rift SDK.
  ORUtils::Vector2<int> depthImageSize = get_model()->get_depth_image_size();
  int width = depthImageSize.width, height = depthImageSize.height;

  ovrPosef eyePoses[ovrEye_Count];
  ovrGLTexture eyeTextures[ovrEye_Count];
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    ovrEyeType eye = m_hmd->EyeRenderOrder[i];
    eyePoses[i] = ovrHmd_GetHmdPosePerEye(m_hmd, eye);  // FIXME: Deprecated.

    eyeTextures[i].OGL.Header.API = ovrRenderAPI_OpenGL;
    eyeTextures[i].OGL.Header.TextureSize = OVR::Sizei(width, height);
    eyeTextures[i].OGL.Header.RenderViewport = OVR::Recti((width - height) / 2, 0, height, height);
    eyeTextures[i].OGL.TexId = m_eyeFrameBuffers[i]->get_colour_buffer_id();
  }

#if 0
  // TODO: Switch to this replacement for ovrHmd_GetHmdPosePerEye.
  //ovrTrackingState trackingState;
  //ovrHmd_GetEyePoses(m_hmd, 0, hmdToEyeViewOffset, eyePoses, &trackingState);
#endif

  // Render the frame and perform a buffer swap.
  ovrHmd_EndFrame(m_hmd, eyePoses, (const ovrTexture *)eyeTextures);
}
