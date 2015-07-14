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

  /** A flag indicating whether or not to use median filtering when rendering the scene raycast. */
  bool m_medianFilteringEnabled;

  /** The spaint model. */
  spaint::SpaintModel_CPtr m_model;

  /** The raycaster to use in order to cast rays into the InfiniTAM scene. */
  spaint::SpaintRaycaster_CPtr m_raycaster;

  /** The type of raycast to use. */
  spaint::SpaintRaycaster::RaycastType m_raycastType;

  /** The ID of a texture in which to temporarily store the scene raycast and touch image when rendering. */
  GLuint m_textureID;

  /** The window into which to render. */
  SDL_Window_Ptr m_window;

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
   * \brief Gets the monocular render state for the camera.
   *
   * If the camera is a stereo one, this will return the render state corresponding to the left eye.
   *
   * \return  The monocular render state for the camera.
   */
  virtual RenderState_CPtr get_monocular_render_state() const = 0;

  /**
   * \brief Gets whether or not the renderer is rendering the scene in mono.
   *
   * \return  true, if the renderer is rendering the scene in mono, or false otherwise.
   */
  virtual bool is_mono() const = 0;

  /**
   * \brief Renders both the reconstructed scene and the synthetic scene from one or more camera poses.
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
   * \brief Gets whether or not to use median filtering when rendering the scene raycast.
   *
   * \return  A flag indicating whether or not to use median filtering when rendering the scene raycast.
   */
  bool get_median_filtering_enabled() const;

  /**
   * \brief Sets the current camera mode.
   *
   * \param cameraMode  The new camera mode.
   */
  void set_camera_mode(CameraMode cameraMode);

  /**
   * \brief Sets whether or not to use median filtering when rendering the scene raycast.
   *
   * \param medianFilteringEnabled  A flag indicating whether or not to use median filtering when rendering the scene raycast.
   */
  void set_median_filtering_enabled(bool medianFilteringEnabled);

  /**
   * \brief Sets the type of raycast to use.
   *
   * \param raycastType The type of raycast to use.
   */
  void set_raycast_type(spaint::SpaintRaycaster::RaycastType raycastType);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Sets appropriate projection and model-view matrices for 2D rendering.
   */
  static void begin_2d();

  /**
   * \brief Destroys the temporary image and texture used for visualising the scene.
   */
  void destroy_common();

  /**
   * \brief Restores the projection and model-view matrices that were active prior to 2D rendering.
   */
  static void end_2d();

  /**
   * \brief Gets the spaint model.
   *
   * \return  The spaint model.
   */
  spaint::SpaintModel_CPtr get_model() const;

  /**
   * \brief Gets the window into which to render.
   *
   * \return  The window into which to render.
   */
  SDL_Window *get_window() const;

  /**
   * \brief Initialises the temporary image and texture used for visualising the scene.
   */
  void initialise_common();

  /**
   * \brief Renders both the reconstructed scene and the synthetic scene from a single camera pose.
   *
   * \param pose        The camera pose.
   * \param interactor  The interactor that is being used to interact with the scene.
   * \param renderState The render state corresponding to the camera pose.
   */
  void render_scene(const ITMPose& pose, const spaint::SpaintInteractor_CPtr& interactor, spaint::SpaintRaycaster::RenderState_Ptr& renderState) const;

  /**
   * \brief Sets the window into which to render.
   *
   * \param window  The window into which to render.
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
   * \brief Renders a quad textured with the specified texture.
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
