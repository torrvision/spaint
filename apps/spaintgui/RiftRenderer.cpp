/**
 * spaintgui: RiftRenderer.cpp
 */

#include "RiftRenderer.h"

#include <stdexcept>

#include <OVR.h>
#include <../Src/OVR_CAPI_GL.h>
#include <../Src/Kernel/OVR_Math.h>

#ifdef __APPLE__
#pragma GCC diagnostic ignored "-Wextern-c-compat"
#endif

#include <SDL_syswm.h>

#include <rigging/DerivedCamera.h>
#include <rigging/SimpleCamera.h>
using namespace rigging;

#include <spaint/util/CameraPoseConverter.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

RiftRenderer::RiftRenderer(const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster,
                           const std::string& title, RiftRenderingMode renderingMode)
: Renderer(model, raycaster)
{
  // Initialise the Rift.
  ovr_Initialize();
  m_hmd = ovrHmd_Create(0);
  if(!m_hmd)
  {
    std::cout << "[spaint] Could not find the Rift, attempting to fall back to a virtual Rift\n";
    m_hmd = ovrHmd_CreateDebug(ovrHmd_DK2);
    if(!m_hmd) throw std::runtime_error("[spaint] Failed to create a virtual Rift!");
  }

  // Create the window into which to render.
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  m_window.reset(
    SDL_CreateWindow(
      title.c_str(),
      renderingMode == WINDOWED_MODE ? SDL_WINDOWPOS_UNDEFINED : SDL_WINDOWPOS_CENTERED_DISPLAY(1),
      renderingMode == WINDOWED_MODE ? SDL_WINDOWPOS_UNDEFINED : SDL_WINDOWPOS_CENTERED_DISPLAY(1),
      m_hmd->Resolution.w,
      m_hmd->Resolution.h,
      renderingMode == WINDOWED_MODE ? SDL_WINDOW_OPENGL : SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN_DESKTOP
    ),
    &SDL_DestroyWindow
  );

  m_context.reset(
    SDL_GL_CreateContext(m_window.get()),
    SDL_GL_DeleteContext
  );

  // Get device-dependent information about the window.
  SDL_SysWMinfo wmInfo;
  SDL_VERSION(&wmInfo.version);
  SDL_GetWindowWMInfo(m_window.get(), &wmInfo);

  // Configure rendering via the Rift SDK.
  const int backBufferMultisample = 1;
  ovrGLConfig cfg;
  cfg.OGL.Header.API = ovrRenderAPI_OpenGL;
  cfg.OGL.Header.RTSize = OVR::Sizei(m_hmd->Resolution.w, m_hmd->Resolution.h);
  cfg.OGL.Header.Multisample = backBufferMultisample;
#ifdef _WIN32
  cfg.OGL.Window = wmInfo.info.win.window;
  cfg.OGL.DC = NULL;
#endif

  const unsigned int distortionCaps = ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette | ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive;
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
  m_camera.reset(new CompositeCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));
  m_camera->add_secondary_camera("left", Camera_CPtr(new DerivedCamera(m_camera, Eigen::Matrix3f::Identity(), Eigen::Vector3f(HALF_IPD, 0.0f, 0.0f))));
  m_camera->add_secondary_camera("right", Camera_CPtr(new DerivedCamera(m_camera, Eigen::Matrix3f::Identity(), Eigen::Vector3f(-HALF_IPD, 0.0f, 0.0f))));

  // Set up the eye images and eye textures.
  ORUtils::Vector2<int> depthImageSize = m_model->get_depth_image_size();
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    m_eyeImages[i].reset(new ITMUChar4Image(depthImageSize, MEMORYDEVICE_CPU));
  }
  glGenTextures(ovrEye_Count, m_eyeTextureIDs);
}

//#################### DESTRUCTOR ####################

RiftRenderer::~RiftRenderer()
{
  glDeleteTextures(ovrEye_Count, m_eyeTextureIDs);
  ovrHmd_Destroy(m_hmd);
  ovr_Shutdown();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

MoveableCamera_Ptr RiftRenderer::get_camera()
{
  return m_camera;
}

void RiftRenderer::render() const
{
  // Keep trying to get rid of the annoying health and safety warning until it goes away.
  ovrHmd_DismissHSWDisplay(m_hmd);

  // Start the frame.
  ovrHmd_BeginFrame(m_hmd, 0);

  // If we're following the reconstruction, update the position and orientation of the camera.
  if(m_cameraMode == CM_FOLLOW)
  {
    m_camera->set_from(CameraPoseConverter::pose_to_camera(m_model->get_pose()));
  }

  // Construct the left and right eye images.
  ITMPose leftPose = CameraPoseConverter::camera_to_pose(*m_camera->get_secondary_camera("left"));
  ITMPose rightPose = CameraPoseConverter::camera_to_pose(*m_camera->get_secondary_camera("right"));
  m_raycaster->generate_free_raycast(m_eyeImages[ovrEye_Left], m_renderStates[ovrEye_Left], leftPose);
  m_raycaster->generate_free_raycast(m_eyeImages[ovrEye_Right], m_renderStates[ovrEye_Right], rightPose);

  // Copy the eye images into OpenGL textures.
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, m_eyeTextureIDs[i]);

    // Invert the image (otherwise it would appear upside-down on the Rift).
    // FIXME: This can be made more efficient.
    Vector4u *imageData = m_eyeImages[i]->GetData(MEMORYDEVICE_CPU);
    Vector4u *invertedImageData = new Vector4u[m_eyeImages[i]->noDims.x * m_eyeImages[i]->noDims.y];
    for(int n = 0; n < m_eyeImages[i]->noDims.x * m_eyeImages[i]->noDims.y; ++n)
    {
      int x = n % m_eyeImages[i]->noDims.x;
      int dy = n / m_eyeImages[i]->noDims.x;
      int sy = m_eyeImages[i]->noDims.y - 1 - dy;
      invertedImageData[n] = imageData[sy * m_eyeImages[i]->noDims.x + x];
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_eyeImages[i]->noDims.x, m_eyeImages[i]->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, invertedImageData);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    delete[] invertedImageData;

    glDisable(GL_TEXTURE_2D);
  }

  // Set up the eye poses and pass the eye textures to the Rift SDK.
  ovrPosef eyePoses[ovrEye_Count];
  ovrGLTexture eyeTextures[ovrEye_Count];
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    ovrEyeType eye = m_hmd->EyeRenderOrder[i];
    eyePoses[i] = ovrHmd_GetHmdPosePerEye(m_hmd, eye);  // FIXME: Deprecated.

    eyeTextures[i].OGL.Header.API = ovrRenderAPI_OpenGL;
    eyeTextures[i].OGL.Header.TextureSize = OVR::Sizei(m_eyeImages[i]->noDims.x, m_eyeImages[i]->noDims.y);
    eyeTextures[i].OGL.Header.RenderViewport = OVR::Recti(OVR::Sizei(m_eyeImages[i]->noDims.x, m_eyeImages[i]->noDims.y));
    eyeTextures[i].OGL.TexId = m_eyeTextureIDs[i];
  }

#if 0
  // TODO: Switch to this replacement for ovrHmd_GetHmdPosePerEye.
  //ovrTrackingState trackingState;
  //ovrHmd_GetEyePoses(m_hmd, 0, hmdToEyeViewOffset, eyePoses, &trackingState);
#endif

  // Render the frame and perform a buffer swap.
  ovrHmd_EndFrame(m_hmd, eyePoses, (const ovrTexture *)eyeTextures);
}
