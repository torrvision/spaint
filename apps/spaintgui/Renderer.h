/**
 * spaintgui: Renderer.h
 */

#ifndef H_SPAINTGUI_RENDERER
#define H_SPAINTGUI_RENDERER

#include <SDL.h>

#include <spaint/SpaintEngine.h>
#include <spaint/cameras/Camera.h>

/**
 * \brief An instance of a class deriving from this one can be used to render the scene constructed by the spaint engine to a given target.
 */
class Renderer
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<void> SDL_GLContext_Ptr;
  typedef boost::shared_ptr<SDL_Window> SDL_Window_Ptr;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The OpenGL context for the window. */
  SDL_GLContext_Ptr m_context;

  /** The spaint engine. */
  spaint::SpaintEngine_Ptr m_spaintEngine;

  /** The window into which to render. */
  SDL_Window_Ptr m_window;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a renderer.
   *
   * \param spaintEngine  The spaint engine.
   */
  explicit Renderer(const spaint::SpaintEngine_Ptr& spaintEngine)
  : m_spaintEngine(spaintEngine)
  {}

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the renderer.
   */
  virtual ~Renderer() {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Renders the scene.
   *
   * \param camera  The camera from which to render the scene (if null, the position of the camera being used for reconstruction is used).
   */
  virtual void render(const spaint::Camera_CPtr& camera) const = 0;
};

#endif
