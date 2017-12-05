/**
 * itmx: GraphVisualiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "graphviz/GraphVisualiser.h"

#include <iostream>

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

namespace bf = boost::filesystem;

//#################### MACROS ####################

// See https://stackoverflow.com/questions/2751870/how-exactly-does-the-double-stringize-trick-work.
#define QUOTE(x) #x
#define STRINGIZE(x) QUOTE(x)

namespace itmx {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void GraphVisualiser::visualise(const std::string& graphDesc, GraphvizExe graphvizExe) const
{
  // Find the graphs directory and make sure it exists.
  bf::path graphsDir = find_subdir_from_executable("graphs");
  bf::create_directories(graphsDir);

  // Write the graph description to a Graphviz source file.
  bf::path tempFile = graphsDir / "temp.gv";

  {
    std::ofstream fs(tempFile.string().c_str());
    fs << graphDesc;
  }

  // Run the chosen Graphviz executable.
  bf::path graphvizExePath = find_path(graphvizExe);
  std::string command = "\"\"" + graphvizExePath.string() + "\" \"" + bf::absolute(tempFile).string() + "\" -Tpng -O\"";
  std::cout << command << '\n';
  int result = system(command.c_str());
  std::cout << result << '\n';
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

bf::path GraphVisualiser::find_path(GraphvizExe graphvizExe) const
{
  bf::path result;

  switch(graphvizExe)
  {
    case GV_DOT:
      result = bf::path(STRINGIZE(GRAPHVIZ_DOT));
      break;
    default:
      result = bf::path(STRINGIZE(GRAPHVIZ_NEATO));
      break;
  }

  if(bf::exists(result)) return result;
  else throw std::runtime_error("Error: Cannot find " + result.string());
}

}
