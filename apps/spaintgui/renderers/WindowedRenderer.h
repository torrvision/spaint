/**
 * spaintgui: WindowedRenderer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_WINDOWEDRENDERER
#define H_SPAINTGUI_WINDOWEDRENDERER

#include <string>

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

  /** The render state for the free camera view. */
  mutable Raycaster::RenderState_Ptr m_renderState;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a windowed renderer.
   *
   * \param title                   The title of the window.
   * \param model                   The spaint model.
   * \param raycaster               The raycaster to use in order to cast rays into the InfiniTAM scene.
   * \param subwindowConfiguration  The sub-window configuration to use for visualising the scene.
   * \param windowViewportSize      The size of the window's viewport.
   */
  WindowedRenderer(const std::string& title, const Model_CPtr& model, const Raycaster_CPtr& raycaster,
                   const SubwindowConfiguration_Ptr& subwindowConfiguration, const Vector2i& windowViewportSize);

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
  virtual bool is_mono() const;

  /** Override */
  virtual void render(const Vector2f& fracWindowPos) const;
};

#endif
