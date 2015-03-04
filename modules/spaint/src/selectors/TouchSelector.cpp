/*
 * spaint: TouchSelector.cpp
 */

#include "selectors/TouchSelector.h"

namespace spaint {

//#################### CONSTRUCTORS #################### 

TouchSelector::TouchSelector(const Settings_CPtr& settings)
: PickingSelector(settings)
{}

//#################### PUBLIC MEMBER FUNCTIONS #################### 

Selector::Selection_CPtr TouchSelector::get_selection() const
{
  std::cout << "This is the touch selector!\n";
  return PickingSelector::get_selection();
}

void TouchSelector::touch_pipeline() const
{
  // The touch pipeline will go here.

  // Update the touch state.
  m_touchState.set_touch_state(250, 250, true, true);
}

void TouchSelector::update(const InputState& inputState, const RenderState_CPtr& renderState)
{
  // Run the touch pipeline.
  touch_pipeline();

  // Update whether or not the selector is active.
  m_isActive = inputState.mouse_button_down(MOUSE_BUTTON_LEFT) || m_touchState.touching_surface();
  const int radius = 5;
  m_radius = radius;

  // Try and pick an individual voxel.
  m_pickPointValid = false;

  if(!m_touchState.touch_position_known()) return;
  int x = m_touchState.position_x();
  int y = m_touchState.position_y();

  //FIXME The following two lines are duplicated from PickingSelector.cpp
  m_pickPointValid = m_picker->pick(x, y, renderState.get(), m_pickPointFloatMB);
  if(m_pickPointValid) m_picker->to_short(m_pickPointFloatMB, m_pickPointShortMB);
}

}
