/**
 * itmx: GraphVisualiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_GRAPHVISUALISER
#define H_ITMX_GRAPHVISUALISER

#include <boost/filesystem.hpp>

namespace itmx {

/**
 * \brief TODO
 */
class GraphVisualiser
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration denote the different Graphviz executables we can use.
   */
  enum GraphvizExe
  {
    GV_DOT,
    GV_NEATO
  };

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   *
   * \param graphDesc   TODO
   * \param graphvizExe The Graphviz executable to use.
   */
  void visualise_graph(const std::string& graphDesc, GraphvizExe graphvizExe = GV_NEATO) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  boost::filesystem::path find_path(GraphvizExe graphvizExe) const;
};

}

#endif
