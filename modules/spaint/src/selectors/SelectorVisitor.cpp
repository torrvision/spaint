/**
 * spaint: SelectorVisitor.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/SelectorVisitor.h"

namespace spaint {

//#################### DESTRUCTOR ####################

SelectorVisitor::~SelectorVisitor() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

#ifdef WITH_LEAP
void SelectorVisitor::visit(const LeapSelector& selector) const {}
#endif
void SelectorVisitor::visit(const NullSelector& selector) const {}
void SelectorVisitor::visit(const PickingSelector& selector) const {}
#ifdef WITH_ARRAYFIRE
void SelectorVisitor::visit(const TouchSelector& selector) const {}
#endif

}
