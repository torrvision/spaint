/**
 * spaintgui: RiftRenderer.h
 */

#ifndef H_SPAINTGUI_RIFTRENDERER
#define H_SPAINTGUI_RIFTRENDERER

#include <string>

#include <OVR.h>

#include <spaint/cameras/CompositeCamera.h>
#include <spaint/ogl/WrappedGL.h>

#include "Renderer.h"

/**
 * \brief An instance of this class can be used to render the scene constructed by the spaint engine to the Oculus Rift.
 */
class RiftRenderer : public Renderer
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration containing the various possible Rift rendering modes.
   */
  enum RiftRenderingMode
  {
    FULLSCREEN_MODE,
    WINDOWED_MODE
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The camera from which to render the scene. */
  spaint::CompositeCamera_Ptr m_camera;

  /** The images in which to store the eye textures each frame. */
  ITMUChar4Image_Ptr m_eyeImages[ovrEye_Count];

  /** The eye texture IDs. */
  GLuint m_eyeTextureIDs[ovrEye_Count];

  /** The Rift handle. */
  ovrHmd m_hmd;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Rift renderer.
   *
   * \param spaintEngine  The spaint engine.
   * \param title         The title to give the window.
   * \param renderingMode The rendering mode to use.
   */
  RiftRenderer(const spaint::SpaintEngine_Ptr& spaintEngine, const std::string& title, RiftRenderingMode renderingMode);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the renderer.
   */
  ~RiftRenderer();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  RiftRenderer(const RiftRenderer&);
  RiftRenderer& operator=(const RiftRenderer&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual spaint::MoveableCamera_Ptr get_camera();

  /** Override */
  virtual void render() const;
};

#endif
