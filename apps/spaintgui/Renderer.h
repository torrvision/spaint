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
  typedef boost::shared_ptr<ITMRenderState> RenderState_CPtr;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The current camera mode. */
  CameraMode m_cameraMode;

  /** The OpenGL context for the window. */
  SDL_GLContext_Ptr m_context;

  /** The interactor that is used to interact with the InfiniTAM scene. */
  spaint::SpaintInteractor_CPtr m_interactor;

  /** The spaint model. */
  spaint::SpaintModel_CPtr m_model;

  /** The raycaster to use in order to cast rays into the InfiniTAM scene. */
  spaint::SpaintRaycaster_CPtr m_raycaster;

  /** The window into which to render. */
  SDL_Window_Ptr m_window;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a renderer.
   *
   * \param model       The spaint model.
   * \param raycaster   The raycaster to use in order to cast rays into the InfiniTAM scene.
   * \param interactor  The interactor that is used to interact with the InfiniTAM scene.
   */
  Renderer(const spaint::SpaintModel_CPtr& model, const spaint::SpaintRaycaster_CPtr& raycaster, const spaint::SpaintInteractor_CPtr& interactor)
  : m_cameraMode(CM_FOLLOW), m_interactor(interactor), m_model(model), m_raycaster(raycaster)
  {}

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the renderer.
   */
  virtual ~Renderer() {}

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
   */
  virtual void render() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the current camera mode.
   *
   * \return  The current camera mode.
   */
  CameraMode get_camera_mode() const
  {
    return m_cameraMode;
  }

  /**
   * \brief Sets the current camera mode.
   *
   * \param cameraMode  The new camera mode.
   */
  void set_camera_mode(CameraMode cameraMode)
  {
    m_cameraMode = cameraMode;
  }
};

#endif
