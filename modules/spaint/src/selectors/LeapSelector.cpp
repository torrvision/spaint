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
  // The Leap coordinate system has x pointing right, y pointing up and z pointing out of the screen, whereas
  // the InfiniTAM coordinate system has x pointing right, y pointing down and z pointing into the screen. As
  // such, we need to flip y and z when converting from the Leap coordinate system to our one. Moreover, the
  // Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the result by 1000.
  return Eigen::Vector3f(leapVec.x, -leapVec.y, -leapVec.z) / 1000;
}

}
