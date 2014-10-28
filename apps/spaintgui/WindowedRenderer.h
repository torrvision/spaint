/**
 * spaintgui: WindowedRenderer.h
 */

#ifndef H_SPAINTGUI_WINDOWEDRENDERER
#define H_SPAINTGUI_WINDOWEDRENDERER

#include <string>

#include "Renderer.h"

/**
 * \brief An instance of this class can be used to render the scene constructed by the spaint engine to a window.
 */
class WindowedRenderer : public Renderer
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a windowed renderer.
   *
   * \param title   The title to give the window.
   * \param width   The width to give the window.
   * \param height  The height to give the window.
   */
  WindowedRenderer(const std::string& title, int width, int height);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void render(const spaint::SpaintEngine_Ptr& spaintEngine) const;
};

#endif
