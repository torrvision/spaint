/**
 * spaintgui: RiftRenderer.h
 */

#ifndef H_SPAINTGUI_RIFTRENDERER
#define H_SPAINTGUI_RIFTRENDERER

#include <string>

#include <OVR.h>

#include <rigging/CompositeCamera.h>

#include <spaint/ogl/FrameBuffer.h>

#include "Renderer.h"

/**
 * \brief An instance of this class can be used to render the spaint scene to the Oculus Rift.
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
  rigging::CompositeCamera_Ptr m_camera;

  /** The eye frame buffers. */
  spaint::FrameBuffer_CPtr m_eyeFrameBuffers[ovrEye_Count];

  /** The Rift handle. */
  ovrHmd m_hmd;

  /** The render states for the two eye views. */
  mutable spaint::SpaintRaycaster::RenderState_Ptr m_renderStates[ovrEye_Count];

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a Rift renderer.
   *
   * \param model         The spaint model.
   * \param raycaster     The raycaster to use in order to cast rays into the InfiniTAM scene.
   * \param title         The title to give the window.
   * \param renderingMode The rendering mode to use.
   */
  RiftRenderer(const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster, const std::string& title, RiftRenderingMode renderingMode);

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
  virtual rigging::MoveableCamera_Ptr get_camera();

  /** Override */
  virtual RenderState_CPtr get_monocular_render_state() const;

  /** Override */
  virtual void render(const spaint::SpaintInteractor_CPtr& interactor) const;
};

#endif
