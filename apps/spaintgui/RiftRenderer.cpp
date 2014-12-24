/**
 * spaintgui: RiftRenderer.cpp
 */

#include "RiftRenderer.h"

#include <stdexcept>

#include <OVR.h>
#include <../Src/OVR_CAPI_GL.h>
#include <../Src/Kernel/OVR_Math.h>
using namespace OVR;

#ifdef __APPLE__
#pragma GCC diagnostic ignored "-Wextern-c-compat"
#endif

#include <SDL_syswm.h>

#include <spaint/cameras/CompositeCamera.h>
#include <spaint/cameras/DerivedCamera.h>
#include <spaint/cameras/SimpleCamera.h>
using namespace spaint;

//#################### CONSTRUCTORS ####################

RiftRenderer::RiftRenderer(const spaint::SpaintEngine_Ptr& spaintEngine, const std::string& title, RiftRenderingMode renderingMode)
: Renderer(spaintEngine)
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
  cfg.OGL.Header.RTSize = Sizei(m_hmd->Resolution.w, m_hmd->Resolution.h);
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

  // Set up the camera.
  MoveableCamera_Ptr primaryCamera(new SimpleCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));
  m_camera.reset(new CompositeCamera(primaryCamera));
  m_camera->add_secondary_camera("left", Camera_CPtr(new DerivedCamera(primaryCamera, Eigen::Matrix4f::Identity())));
  m_camera->add_secondary_camera("right", Camera_CPtr(new DerivedCamera(primaryCamera, Eigen::Matrix4f::Identity())));

  // Set up the eye images and eye textures.
  ITMLib::Vector2<int> depthImageSize = spaintEngine->get_image_source_engine()->getDepthImageSize();
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    m_eyeImages[i].reset(new ITMUChar4Image(depthImageSize, false));
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

spaint::MoveableCamera_Ptr RiftRenderer::get_camera()
{
  return m_camera;
}

void RiftRenderer::render() const
{
  // Keep trying to get rid of the annoying health and safety warning until it goes away.
  ovrHmd_DismissHSWDisplay(m_hmd);

  // Start the frame.
  ovrHmd_BeginFrame(m_hmd, 0);

  // Construct the left and right eye images.
  m_spaintEngine->get_default_raycast(m_eyeImages[ovrEye_Left]);
  m_spaintEngine->get_default_raycast(m_eyeImages[ovrEye_Right]);

  // Copy the eye images into OpenGL textures.
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, m_eyeTextureIDs[i]);

    // Invert the image (otherwise it would appear upside-down on the Rift).
    // FIXME: This can be made more efficient.
    Vector4u *imageData = m_eyeImages[i]->GetData(false);
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
    eyeTextures[i].OGL.Header.TextureSize = Sizei(m_eyeImages[i]->noDims.x, m_eyeImages[i]->noDims.y);
    eyeTextures[i].OGL.Header.RenderViewport = Recti(Sizei(m_eyeImages[i]->noDims.x, m_eyeImages[i]->noDims.y));
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
