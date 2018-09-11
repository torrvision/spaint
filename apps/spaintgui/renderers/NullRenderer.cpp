/**
 * spaintgui: NullRenderer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "NullRenderer.h"

//#################### CONSTRUCTORS ####################

NullRenderer::NullRenderer(const Model_CPtr& model, bool verbose)
: Renderer(model, SubwindowConfiguration::make_default(1, Vector2i(640, 480), ""), Vector2i(640, 480)),
  m_frameIdx(0),
  m_verbose(verbose)
{
  // Note: we initialise a dummy subwindow configuration with a fixed size, but it will never be used in all practical scenarios.
}

//#################### DESTRUCTOR ####################

NullRenderer::~NullRenderer()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

VoxelRenderState_CPtr NullRenderer::get_monocular_render_state(size_t subwindowIndex) const
{
  return get_subwindow_configuration()->subwindow(subwindowIndex).get_voxel_render_state();
}

bool NullRenderer::is_mono() const
{
  return true;
}

void NullRenderer::render(const Vector2f& fracWindowPos, bool renderFiducials) const
{
  if(m_verbose)
  {
    std::cout << "\rProcessing frame: " << m_frameIdx << std::flush;
  }

  m_frameIdx++;
}
