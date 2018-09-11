/**
 * spaintgui: NullRenderer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_SPAINTGUI_NULLRENDERER
#define H_SPAINTGUI_NULLRENDERER

#include "Renderer.h"

/**
 * \brief An instance of this class can be used when running spaintgui in headless mode, removing the requirements for a GUI environment.
 *        The whole rendering phase is skipped, thus also speeding up the application.
 */
class NullRenderer : public Renderer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The index of the currently rendered frame. */
  mutable uint m_frameIdx;

  /** Whether or not the rendering is verbose (prints the current frame index). */
  bool m_verbose;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a NullRenderer.
   *
   * \param model    The spaint model.
   * \param verbose  Whether or not to print the current frame index when processing the render call.
   */
  NullRenderer(const Model_CPtr& model, bool verbose = false);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the renderer.
   */
  ~NullRenderer();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  NullRenderer(const NullRenderer&);
  NullRenderer& operator=(const NullRenderer&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual VoxelRenderState_CPtr get_monocular_render_state(size_t subwindowIndex) const;

  /** Override */
  virtual bool is_mono() const;

  /** Override */
  virtual void render(const Vector2f& fracWindowPos, bool renderFiducials) const;
};

#endif
