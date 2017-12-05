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
  /** The random number generator needed for creating UUIDs. */
  boost::uuids::random_generator m_gen;

  /** The synchronisation mutex. */
  boost::mutex m_mutex;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   *
   * \param graphDesc   TODO
   * \param graphvizExe The Graphviz executable to use.
   */
  ITMUChar4Image_Ptr generate_visualisation(const std::string& graphDesc, GraphvizExe graphvizExe = GV_NEATO);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  boost::filesystem::path find_path(GraphvizExe graphvizExe) const;

  /**
   * \brief Makes a UUID.
   *
   * \return  The UUID.
   */
  std::string make_uuid();
};

}

#endif
