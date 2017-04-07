/**
 * spaint: PickingSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/PickingSelector.h"
using namespace tvginput;

#include <itmx/MemoryBlockFactory.h>
using itmx::MemoryBlockFactory;

#include "picking/PickerFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

PickingSelector::PickingSelector(const Settings_CPtr& settings)
: Selector(settings),
  m_picker(PickerFactory::make_picker(settings->deviceType)),
  m_pickPointFloatMB(MemoryBlockFactory::instance().make_block<Vector3f>(1)),
  m_pickPointShortMB(MemoryBlockFactory::instance().make_block<Vector3s>(1)),
  m_pickPointValid(false)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PickingSelector::accept(const SelectorVisitor& visitor) const
{
  visitor.visit(*this);
}

boost::optional<Eigen::Vector3f> PickingSelector::get_position() const
{
  // If the last update did not yield a valid pick point, early out.
  if(!m_pickPointValid) return boost::none;

  // Convert the pick point from voxel coordinates into scene coordinates and return it.
  return Picker::get_positions<Eigen::Vector3f>(*m_pickPointFloatMB, m_settings->sceneParams.voxelSize)[0];
}

Selector::Selection_CPtr PickingSelector::get_selection() const
{
  return m_pickPointValid ? m_pickPointShortMB : Selection_CPtr();
}

void PickingSelector::update(const InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono)
{
  // Update whether or not the selector is active.
  m_isActive = renderingInMono && inputState.mouse_button_down(MOUSE_BUTTON_LEFT);

  // If the scene is not being rendered in mono, early out.
  if(!renderingInMono) return;

  // Try to pick an individual voxel.
  m_pickPointValid = false;

  if(!inputState.mouse_position_known()) return;
  int x = (int)ROUND(inputState.mouse_position_x() * (renderState->raycastResult->noDims.x - 1));
  int y = (int)ROUND(inputState.mouse_position_y() * (renderState->raycastResult->noDims.y - 1));

  m_pickPointValid = m_picker->pick(x, y, renderState.get(), *m_pickPointFloatMB);
  if(m_pickPointValid) m_picker->to_short(*m_pickPointFloatMB, *m_pickPointShortMB);
}

}
