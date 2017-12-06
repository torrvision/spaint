/**
 * itmx: GraphVisualiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_GRAPHVISUALISER
#define H_ITMX_GRAPHVISUALISER

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "../base/ITMImagePtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to generate graph visualisations using Graphviz.
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
  /** The random number generator needed for creating UUIDs. */
  boost::uuids::random_generator m_gen;

  /** The synchronisation mutex. */
  boost::mutex m_mutex;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates a graph visualisation by running a Graphviz executable on the specified graph description.
   *
   * \param graphDesc   The graph description (i.e. the contents of the Graphviz source file).
   * \param graphvizExe The Graphviz executable to use.
   */
  ITMUChar4Image_Ptr generate_visualisation(const std::string& graphDesc, GraphvizExe graphvizExe = GV_NEATO);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the path to the specified Graphviz executable.
   *
   * \param graphvizExe The Graphviz executable whose path we want to get.
   * \return            The path to the specified Graphviz executable.
   */
  boost::filesystem::path get_path(GraphvizExe graphvizExe) const;

  /**
   * \brief Makes a UUID.
   *
   * \return  The UUID (in string form).
   */
  std::string make_uuid();
};

}

#endif
