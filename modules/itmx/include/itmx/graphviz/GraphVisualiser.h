/**
 * itmx: GraphVisualiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_GRAPHVISUALISER
#define H_ITMX_GRAPHVISUALISER

#include <boost/shared_ptr.hpp>

#include <graphviz/gvc.h>

namespace itmx {

/**
 * \brief TODO
 */
class GraphVisualiser
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration denote the different layout engines that can be used.
   */
  enum LayoutEngine
  {
    LE_DOT,
    LE_NEATO
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The Graphviz context. */
  boost::shared_ptr<GVC_t> m_context;

  /** The layout engine to use. */
  LayoutEngine m_layoutEngine;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a graph visualiser.
   *
   * \param layoutEngine  The layout engine to use.
   */
  explicit GraphVisualiser(LayoutEngine layoutEngine);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  void visualise(const std::string& graphDesc) const;
};

}

#endif
