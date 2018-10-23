/**
 * spaintgui: HeadlessRenderer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_SPAINTGUI_HEADLESSRENDERER
#define H_SPAINTGUI_HEADLESSRENDERER

#include "Renderer.h"

/**
 * \brief An instance of this class can be used to run spaintgui in headless mode, removing the need for a GUI environment.
 *        Rendering is skipped, which also makes reconstruction faster.
 */
class HeadlessRenderer : public Renderer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The index of the next frame that will be rendered. */
  mutable uint m_frameIdx;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a headless renderer.
   *
   * \param model    The spaint model.
   */
  explicit HeadlessRenderer(const Model_CPtr& model);

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  HeadlessRenderer(const HeadlessRenderer&);
  HeadlessRenderer& operator=(const HeadlessRenderer&);

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
