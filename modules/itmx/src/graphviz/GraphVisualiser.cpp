/**
 * itmx: GraphVisualiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "graphviz/GraphVisualiser.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

GraphVisualiser::GraphVisualiser(LayoutEngine layoutEngine)
: m_context(gvContext(), gvFreeContext),
  m_layoutEngine(layoutEngine)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void GraphVisualiser::visualise(const std::string& graphDesc) const
{
  boost::shared_ptr<Agraph_t> g(agmemread(graphDesc.c_str()), agclose);

}

}
