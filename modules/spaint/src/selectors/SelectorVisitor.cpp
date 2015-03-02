/**
 * spaint: SelectorVisitor.cpp
 */

#include "selectors/SelectorVisitor.h"

namespace spaint {

//#################### DESTRUCTOR ####################

SelectorVisitor::~SelectorVisitor() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SelectorVisitor::visit(const NullSelector& selector) const {}
void SelectorVisitor::visit(const PickingSelector& selector) const {}

}
