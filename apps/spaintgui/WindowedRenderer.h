/**
 * spaintgui: WindowedRenderer.h
 */

#ifndef H_SPAINTGUI_WINDOWEDRENDERER
#define H_SPAINTGUI_WINDOWEDRENDERER

#include <string>

#include <spaint/ogl/WrappedGL.h>

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

  /** The image in which to store the visualisation each frame. */
  ITMUChar4Image_Ptr m_image;

  /** The render state for the free camera view. */
  mutable spaint::SpaintRaycaster::RenderState_Ptr m_renderState;

  /** The texture ID for the visualisation we're drawing. */
  GLuint m_textureID;

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
  virtual void render(const spaint::Selector_CPtr& selector) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Sets appropriate projection and model-view matrices for 2D rendering.
   */
  static void begin_2d();

  /**
   * \brief Restores the projection and model-view matrices that were active prior to 2D rendering.
   */
  static void end_2d();

  /**
   * \brief Renders the reconstructed scene.
   *
   * \param pose  The camera pose.
   */
  void render_reconstructed_scene(const ITMPose& pose) const;

  /**
   * \brief Renders a synthetic scene to augment what actually exists in the real world.
   *
   * \param pose      The camera pose.
   * \param selector  The selector that is being used to select voxels in the InfiniTAM scene.
   */
  void render_synthetic_scene(const ITMPose& pose, const spaint::Selector_CPtr& selector) const;

  /**
   * \brief Sets the OpenGL projection matrix based on a set of intrinsic camera parameters.
   *
   * \param intrinsics  The intrinsic camera parameters.
   * \param width       The width of the viewport.
   * \param height      The height of the viewport.
   */
  static void set_projection_matrix(const ITMIntrinsics& intrinsics, int width, int height);

  //#################### FRIENDS ####################

  friend class SelectorRenderer;
};

#endif
