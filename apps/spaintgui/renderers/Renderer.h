/**
 * spaintgui: Renderer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_RENDERER
#define H_SPAINTGUI_RENDERER

// Prevent SDL from trying to define M_PI.
#define HAVE_M_PI

#include <SDL.h>

#include <rigging/MoveableCamera.h>

#include <spaint/ogl/WrappedGL.h>

#include "../core/Interactor.h"
#include "../subwindows/SubwindowConfiguration.h"

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
  typedef boost::shared_ptr<const ITMLib::ITMRenderState> RenderState_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The current camera mode. */
  CameraMode m_cameraMode;

  /** The OpenGL context for the window. */
  SDL_GLContext_Ptr m_context;

  /** A flag indicating whether or not to use median filtering when rendering the scene raycast. */
  bool m_medianFilteringEnabled;

  /** The spaint model. */
  Model_CPtr m_model;

  /** The raycaster to use in order to cast rays into the InfiniTAM scene. */
  Raycaster_CPtr m_raycaster;

  /** A set of saved sub-window configurations that the user can switch between as desired. */
  std::vector<SubwindowConfiguration_Ptr> m_savedSubwindowConfigurations;

  /** The current sub-window configuration. */
  SubwindowConfiguration_Ptr m_subwindowConfiguration;

  /** The ID of a texture in which to temporarily store the scene raycast and touch image when rendering. */
  GLuint m_textureID;

  /** The size of the window's viewport. */
  Vector2i m_viewportSize;

  /** The window into which to render. */
  SDL_Window_Ptr m_window;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a renderer.
   *
   * \param model         The spaint model.
   * \param raycaster     The raycaster to use in order to cast rays into the InfiniTAM scene.
   * \param viewportSize  The size of the window's viewport.
   */
  Renderer(const Model_CPtr& model, const Raycaster_CPtr& raycaster, const boost::optional<Vector2i>& viewportSize = boost::none);

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
  virtual void render(const Interactor_CPtr& interactor) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Computes the fractional position of point (x,y) in the viewport.
   *
   * \param x The x coordinate of the point whose fractional position is to be computed.
   * \param y The y coordinate of the point whose fractional position is to be computed.
   * \return  The fractional position of the specified point in the viewport.
   */
  Vector2f compute_fractional_viewport_position(int x, int y) const;

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
   * \brief Gets the active sub-window configuration.
   *
   * \return  The active sub-window configuration.
   */
  SubwindowConfiguration_CPtr get_subwindow_configuration() const;

  /**
   * \brief Resets the current sub-window configuration to its default settings.
   */
  void reset_subwindow_configuration();

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
   * \brief Sets the sub-window configuration to use for visualising the scene.
   *
   * \param i The index of the sub-window configuration to use for visualising the scene.
   */
  void set_subwindow_configuration(size_t i);

  /**
   * \brief Sets the type of a sub-window that is being used to visualise the scene.
   *
   * \param subwindowIndex  The index of the subwindow whose type is to be set.
   * \param type            The new type of visualisation to use.
   */
  void set_subwindow_type(size_t subwindowIndex, Raycaster::RaycastType type);

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
  Model_CPtr get_model() const;

  /**
   * \brief Gets the size of the window's viewport.
   *
   * \return  The size of the window's viewport.
   */
  const Vector2i& get_viewport_size() const;

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
  void render_scene(const ORUtils::SE3Pose& pose, const Interactor_CPtr& interactor, Raycaster::RenderState_Ptr& renderState) const;

  /**
   * \brief Sets the window into which to render.
   *
   * \param window  The window into which to render.
   */
  void set_window(const SDL_Window_Ptr& window);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Renders the reconstructed scene into a sub-window.
   *
   * \param pose            The camera pose.
   * \param renderState     The render state corresponding to the camera pose.
   * \param subwindowIndex  The index of the sub-window into which to render.
   */
  void render_reconstructed_scene(const ORUtils::SE3Pose& pose, Raycaster::RenderState_Ptr& renderState, size_t subwindowIndex) const;

  /**
   * \brief Renders a synthetic scene into a sub-window to augment what actually exists in the real world.
   *
   * \param pose            The camera pose.
   * \param interactor      The interactor that is being used to interact with the scene.
   * \param subwindowIndex  The index of the sub-window into which to render.
   */
  void render_synthetic_scene(const ORUtils::SE3Pose& pose, const Interactor_CPtr& interactor, size_t subwindowIndex) const;

  /**
   * \brief Renders a quad textured with the specified texture.
   *
   * \param textureID   The ID of the texture to apply to the quad.
   */
  static void render_textured_quad(GLuint textureID);

  /**
   * \brief Sets the OpenGL projection matrix based on a set of intrinsic camera parameters.
   *
   * \param intrinsics  The intrinsic camera parameters.
   * \param width       The width of the viewport.
   * \param height      The height of the viewport.
   */
  static void set_projection_matrix(const ITMLib::ITMIntrinsics& intrinsics, int width, int height);

  //#################### FRIENDS ####################

  friend class SelectorRenderer;
};

#endif
