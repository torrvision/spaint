/**
 * spaint: TouchState.cpp
 */

#include "touch/TouchState.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchState::TouchState()
: m_touchingSurface(false), m_touchPositionKnown(false)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const std::vector<int>& TouchState::position_x() const
{
  return m_position_x;
}

const std::vector<int>& TouchState::position_y() const
{
  return m_position_y;
}

void TouchState::set_touch_state(const std::vector<int>& position_x, const std::vector<int>& position_y, bool touchingSurface, bool touchPositionKnown)
{
  m_position_x = position_x;
  m_position_y = position_y;
  m_touchingSurface = touchingSurface;
  m_touchPositionKnown = touchPositionKnown;
}

bool TouchState::touching_surface() const
{
  return m_touchingSurface;
}

bool TouchState::touch_position_known() const
{
  return m_touchPositionKnown;
}
}

