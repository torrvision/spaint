/**
 * itmx: GraphVisualiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "graphviz/GraphVisualiser.h"

namespace bf = boost::filesystem;

namespace itmx {

//#################### CONSTRUCTORS ####################

GraphVisualiser::GraphVisualiser(GraphvizExe exe)
{
  switch(exe)
  {
    case GV_DOT:
      m_exe = bf::path("GRAPHVIZ_DOT");
      break;
    default:
      m_exe = bf::path("GRAPHVIZ_NEATO");
      break;
  }

  if(!bf::exists(m_exe))
  {
    throw std::runtime_error("Error: Graphviz executable '" + m_exe.string() + "' is missing");
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void GraphVisualiser::visualise(const std::string& graphDesc) const
{

}

}
