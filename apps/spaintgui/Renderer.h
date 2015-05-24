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

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The current camera mode. */
  CameraMode m_cameraMode;

  /** The OpenGL context for the window. */
  SDL_GLContext_Ptr m_context;

  /** The spaint model. */
  spaint::SpaintModel_CPtr m_model;

  /** Whether or not Phong lighting is currently enabled. */
  bool m_phongEnabled;

  /** The raycaster to use in order to cast rays into the InfiniTAM scene. */
  spaint::SpaintRaycaster_CPtr m_raycaster;

  /** The window into which to render. */
  SDL_Window_Ptr m_window;

  //#################### CONSTRUCTORS ####################
public:
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
};

#endif
