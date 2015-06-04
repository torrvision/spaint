/**
 * spaint: TouchState.cpp
 */

#include "touch/TouchState.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

TouchState::TouchState()
: m_touchingSurface(false), m_touchPositionKnown(false)
{}

TouchState::TouchState(const TouchState::TouchPositions_CPtr& touchPositions, bool touchingSurface, bool touchPositionKnown)
: m_touchPositions(touchPositions), m_touchingSurface(touchingSurface), m_touchPositionKnown(touchPositionKnown)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

TouchState::TouchPositions_CPtr TouchState::get_positions() const
{
  return m_touchPositions;
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
