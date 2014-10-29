/**
 * spaintgui: RiftRenderer.cpp
 */

#include "RiftRenderer.h"

#include <stdexcept>

#include <OVR.h>
#include <../Src/OVR_CAPI_GL.h>
#include <../Src/Kernel/OVR_Math.h>
using namespace OVR;

#include <SDL_syswm.h>

//#################### CONSTRUCTORS ####################

RiftRenderer::RiftRenderer(const std::string& title)
{
  // Initialise the Rift.
  ovr_Initialize();
  m_hmd = ovrHmd_Create(0);
  if(!m_hmd) throw std::runtime_error("[spaint] Could not find the Rift!");

  // Configure and start the sensor that provides the Rift's pose and motion.
  ovrHmd_ConfigureTracking(m_hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);

  // Recentre the pose.
  ovrHmd_RecenterPose(m_hmd);

  // Create the window into which to render.
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  m_window.reset(
    SDL_CreateWindow(
      title.c_str(),
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      m_hmd->Resolution.w,
      m_hmd->Resolution.h,
      SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN
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
}

//#################### DESTRUCTOR ####################

RiftRenderer::~RiftRenderer()
{
  ovrHmd_Destroy(m_hmd);
  ovr_Shutdown();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void RiftRenderer::render(const spaint::SpaintEngine_Ptr& spaintEngine) const
{
  ITMLib::Vector2<int> depthImageSize = spaintEngine->get_image_source_engine()->getDepthImageSize();

  static spaint::shared_ptr<ITMUChar4Image> rgbImage(new ITMUChar4Image(depthImageSize, false));
  static GLuint textureID;
  static bool done = false;
  if(!done)
  {
    glGenTextures(1, &textureID);
    done = true;
  }

  ovrHmd_BeginFrame(m_hmd, 0);

  spaintEngine->get_default_raycast(rgbImage);

  glEnable(GL_TEXTURE_2D);
  {
    glBindTexture(GL_TEXTURE_2D, textureID);

#if 1
    // Invert the image (otherwise it appears upside-down on the Rift).
    Vector4u *p = rgbImage->GetData(false);
    Vector4u *q = new Vector4u[rgbImage->noDims.x * rgbImage->noDims.y];
    for(int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; ++i)
    {
      int x = i % rgbImage->noDims.x;
      int dy = i / rgbImage->noDims.x;
      int sy = rgbImage->noDims.y - 1 - dy;
      q[i] = p[sy * rgbImage->noDims.x + x];
    }
#endif

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgbImage->noDims.x, rgbImage->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, q);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

#if 1
    delete[] q;
#endif
  }
  glDisable(GL_TEXTURE_2D);

  ovrPosef eyePoses[ovrEye_Count];
  ovrGLTexture eyeTextures[ovrEye_Count];
  for(int i = 0; i < ovrEye_Count; ++i)
  {
    ovrEyeType eye = m_hmd->EyeRenderOrder[i];
    eyePoses[i] = ovrHmd_GetHmdPosePerEye(m_hmd, eye);  // FIXME: Deprecated.

    eyeTextures[i].OGL.Header.API = ovrRenderAPI_OpenGL;
    eyeTextures[i].OGL.Header.TextureSize = Sizei(depthImageSize.x, depthImageSize.y);
    eyeTextures[i].OGL.Header.RenderViewport = Recti(Sizei(depthImageSize.x, depthImageSize.y));
    eyeTextures[i].OGL.TexId = textureID;
  }

#if 0
  // TODO: Switch to this replacement for ovrHmd_GetHmdPosePerEye.
  //ovrTrackingState trackingState;
  //ovrHmd_GetEyePoses(m_hmd, 0, hmdToEyeViewOffset, eyePoses, &trackingState);
#endif

  ovrHmd_EndFrame(m_hmd, eyePoses, (const ovrTexture *)eyeTextures);
}