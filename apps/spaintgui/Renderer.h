/**
 * spaintgui: Renderer.h
 */

#ifndef H_SPAINTGUI_RENDERER
#define H_SPAINTGUI_RENDERER

#include <SDL.h>

#include <rigging/MoveableCamera.h>

#include <spaint/core/SpaintInteractor.h>
#include <spaint/core/SpaintModel.h>
#include <spaint/core/SpaintRaycaster.h>
#include <spaint/ogl/WrappedGL.h>

/**
 * \brief An instance of a class deriving from this one can be used to render the spaint scene to a given target.
 */
class Renderer
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration containing the possible camera modes we can use.
   */
  enum CameraMode
  {
    /** A mode that follows the camera that is reconstructing the scene. */
    CM_FOLLOW,

    /** A mode that allows the user to freely move the camera around to view the scene from different angles. */
    CM_FREE
  };

  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<void> SDL_GLContext_Ptr;
  typedef boost::shared_ptr<SDL_Window> SDL_Window_Ptr;
public:
  typedef boost::shared_ptr<const ITMRenderState> RenderState_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The current camera mode. */
  CameraMode m_cameraMode;

  /** The OpenGL context for the window. */
  SDL_GLContext_Ptr m_context;

  /** An image in which to temporarily store visualisations of the scene. */
  ITMUChar4Image_Ptr m_image;

  /** Whether or not Phong lighting is currently enabled. */
  bool m_phongEnabled;

  /** The raycaster to use in order to cast rays into the InfiniTAM scene. */
  spaint::SpaintRaycaster_CPtr m_raycaster;

  /** The texture ID for the visualisation we're drawing. */
  GLuint m_textureID;

  /** The window into which to render. */
  SDL_Window_Ptr m_window;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The spaint model. */
  spaint::SpaintModel_CPtr m_model;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a renderer.
   *
   * \param model     The spaint model.
   * \param raycaster The raycaster to use in order to cast rays into the InfiniTAM scene.
   */
  Renderer(const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the renderer.
   */
  virtual ~Renderer();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the camera from which to render the scene.
   *
   * \return  The camera from which to render the scene.
   */
  virtual rigging::MoveableCamera_Ptr get_camera() = 0;

  /**
   * \brief Gets the render state for the camera, if it's monocular, or null otherwise.
   *
   * \return  The render state for the camera, if it's monocular, or null otherwise.
   */
  virtual RenderState_CPtr get_monocular_render_state() const = 0;

  /**
   * \brief Renders the scene.
   *
   * \param interactor  The interactor that is being used to interact with the scene.
   */
  virtual void render(const spaint::SpaintInteractor_CPtr& interactor) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the current camera mode.
   *
   * \return  The current camera mode.
   */
  CameraMode get_camera_mode() const;

  /**
   * \brief Gets whether or not Phong lighting is currently enabled.
   *
   * \return  true, if Phong lighting is currently enabled, or false otherwise.
   */
  bool get_phong_enabled() const;

  /**
   * \brief Sets the current camera mode.
   *
   * \param cameraMode  The new camera mode.
   */
  void set_camera_mode(CameraMode cameraMode);

  /**
   * \brief Enables or disables Phong lighting.
   *
   * \param phongEnabled  true, if Phong lighting should be enabled, or false otherwise.
   */
  void set_phong_enabled(bool phongEnabled);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Sets appropriate projection and model-view matrices for 2D rendering.
   */
  static void begin_2d();

  /**
   * \brief TODO
   */
  void destroy_common();

  /**
   * \brief Restores the projection and model-view matrices that were active prior to 2D rendering.
   */
  static void end_2d();

  /**
   * \brief TODO
   */
  SDL_Window *get_window() const;

  /**
   * \brief TODO
   */
  void initialise_common();

  /**
   * \brief Renders the scene.
   *
   * \param pose        The camera pose.
   * \param interactor  The interactor that is being used to interact with the scene.
   * \param renderState The render state corresponding to the camera pose.
   */
  void render_scene(const ITMPose& pose, const spaint::SpaintInteractor_CPtr& interactor, spaint::SpaintRaycaster::RenderState_Ptr& renderState) const;

  /**
   * \brief TODO
   */
  void set_window(const SDL_Window_Ptr& window);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Renders the reconstructed scene.
   *
   * \param pose        The camera pose.
   * \param renderState The render state corresponding to the camera pose.
   */
  void render_reconstructed_scene(const ITMPose& pose, spaint::SpaintRaycaster::RenderState_Ptr& renderState) const;

  /**
   * \brief Renders a synthetic scene to augment what actually exists in the real world.
   *
   * \param pose        The camera pose.
   * \param interactor  The interactor that is being used to interact with the scene.
   */
  void render_synthetic_scene(const ITMPose& pose, const spaint::SpaintInteractor_CPtr& interactor) const;

  /**
   * \brief Renders a quad textured with the specified texture into a frame buffer.
   *
   * \param textureID The ID of the texture to apply to the quad.
   */
  static void render_textured_quad(GLuint textureID);

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
