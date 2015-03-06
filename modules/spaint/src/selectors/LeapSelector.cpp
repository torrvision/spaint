/**
 * spaint: LeapSelector.cpp
 */

#include "selectors/LeapSelector.h"

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LeapSelector::accept(const SelectorVisitor& visitor) const
{
  visitor.visit(*this);
}

const Leap::Frame& LeapSelector::get_frame() const
{
  return m_frame;
}

Selector::Selection_CPtr LeapSelector::get_selection() const
{
  // TODO
  return Selection_CPtr();
}

void LeapSelector::update(const InputState& inputState, const RenderState_CPtr& renderState)
{
  m_frame = m_leap.frame();
  if(!m_frame.isValid() || m_frame.hands().count() != 1) return;

  // TODO
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Eigen::Vector3f LeapSelector::from_leap_vector(const Leap::Vector& leapVec)
{
  // TODO: Write an explanatory comment.
  return Eigen::Vector3f(leapVec.x / 1000, -leapVec.y / 1000, -leapVec.z / 1000);
}

}
