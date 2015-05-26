/**
 * spaintgui: WindowedRenderer.h
 */

#ifndef H_SPAINTGUI_WINDOWEDRENDERER
#define H_SPAINTGUI_WINDOWEDRENDERER

#include <string>

#include <spaint/ogl/FrameBuffer.h>

#include "Renderer.h"

/**
 * \brief An instance of this class can be used to render the spaint scene to a window.
 */
class WindowedRenderer : public Renderer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The camera from which to render the scene. */
  rigging::MoveableCamera_Ptr m_camera;

  /** The height of the window. */
  int m_height;

  /** The render state for the free camera view. */
  mutable spaint::SpaintRaycaster::RenderState_Ptr m_renderState;

  /** The width of the window. */
  int m_width;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a windowed renderer.
   *
   * \param model     The spaint model.
   * \param raycaster The raycaster to use in order to cast rays into the InfiniTAM scene.
   * \param title     The title of the window.
   * \param width     The width of the window.
   * \param height    The height of the window.
   */
  WindowedRenderer(const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster, const std::string& title, int width, int height);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the renderer.
   */
  ~WindowedRenderer();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  WindowedRenderer(const WindowedRenderer&);
  WindowedRenderer& operator=(const WindowedRenderer&);

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
