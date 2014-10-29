/**
 * spaintgui: Renderer.h
 */

#ifndef H_SPAINTGUI_RENDERER
#define H_SPAINTGUI_RENDERER

#include <SDL.h>

#include <spaint/SpaintEngine.h>

/**
 * \brief An instance of a class deriving from this one can be used to render the scene constructed by the spaint engine to a given target.
 */
class Renderer
{
  //#################### TYPEDEFS ####################
protected:
  typedef spaint::shared_ptr<void> SDL_GLContext_Ptr;
  typedef spaint::shared_ptr<SDL_Window> SDL_Window_Ptr;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The OpenGL context for the window. */
  SDL_GLContext_Ptr m_context;

  /** The window into which to render. */
  SDL_Window_Ptr m_window;

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
   * \param spaintEngine  The spaint engine.
   */
  virtual void render(const spaint::SpaintEngine_Ptr& spaintEngine) const = 0;
};

#endif
