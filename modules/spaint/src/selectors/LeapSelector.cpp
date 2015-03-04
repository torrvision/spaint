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

Selector::Selection_CPtr LeapSelector::get_selection() const
{
  // TODO
  throw 23;
}

void LeapSelector::update(const InputState& inputState, const RenderState_CPtr& renderState)
{
  m_frame = m_leap.frame();
  if(!m_frame.isValid() || m_frame.hands().count() != 1) return;

  // TODO
  throw 23;
}

}
