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

  //#################### PRIVATE VARIABLES ####################
private:
  /** The executable to use. */
  boost::filesystem::path m_exe;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a graph visualiser.
   *
   * \param exeName The Graphviz executable to use.
   */
  explicit GraphVisualiser(GraphvizExe exe = GV_NEATO);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  void visualise(const std::string& graphDesc) const;
};

}

#endif
