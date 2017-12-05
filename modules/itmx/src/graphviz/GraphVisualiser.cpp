/**
 * itmx: GraphVisualiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "graphviz/GraphVisualiser.h"

#include <fstream>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

#include "persistence/ImagePersister.h"

//#################### MACROS ####################

// See https://stackoverflow.com/questions/2751870/how-exactly-does-the-double-stringize-trick-work.
#define QUOTE(x) #x
#define STRINGIZE(x) QUOTE(x)

//#################### NAMESPACE ALIASES ####################

namespace bf = boost::filesystem;

namespace itmx {

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMUChar4Image_Ptr GraphVisualiser::generate_visualisation(const std::string& graphDesc, GraphvizExe graphvizExe)
{
  std::string stem;

  // Find the graphs directory.
  const bf::path graphsDir = find_subdir_from_executable("graphs");

  {
    boost::lock_guard<boost::mutex> lock(m_mutex);

    // Make sure the graphs directory exists.
    bf::create_directories(graphsDir);

    // Generate a unique stem.
    stem = make_uuid();
  }

  // Write the graph description to a Graphviz source file whose name is based on the generated stem.
  const bf::path sourceFile = graphsDir / (stem + ".gv");

  {
    std::ofstream fs(sourceFile.string().c_str());
    fs << graphDesc;
  }

  // Run the chosen Graphviz executable to generate the graph image.
  bf::path graphvizExePath = get_path(graphvizExe);
#ifdef _WIN32
  std::string command = "\"\"" + graphvizExePath.string() + "\" \"" + bf::absolute(sourceFile).string() + "\" -Tpng -O\"";
#else
  std::string command = graphvizExePath.string() + " \"" + bf::absolute(sourceFile).string() + "\" -Tpng -O";
#endif
  int result = system(command.c_str());
  if(result != 0) throw std::runtime_error("Error: Graphviz execution failed");

  // Read the graph image back in.
  const bf::path graphImageFile = graphsDir / (stem + ".gv.png");
  ITMUChar4Image_Ptr graphImage = ImagePersister::load_rgba_image(graphImageFile.string());

  // Finally, delete the source and graph image files to keep things tidy.
  bf::remove(sourceFile);
  bf::remove(graphImageFile);

  return graphImage;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

bf::path GraphVisualiser::get_path(GraphvizExe graphvizExe) const
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

std::string GraphVisualiser::make_uuid()
{
  return boost::lexical_cast<std::string>(m_gen());
}

}
