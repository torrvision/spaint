/**
 * spaint: NullSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/NullSelector.h"
using namespace tvginput;

namespace spaint {

//#################### CONSTRUCTORS ####################

NullSelector::NullSelector(const Settings_CPtr& settings)
: Selector(settings)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void NullSelector::accept(const SelectorVisitor& visitor) const
{
  visitor.visit(*this);
}

Selector::Selection_CPtr NullSelector::get_selection() const
{
  return Selection_CPtr();
}

void NullSelector::update(const InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono) {}

}
