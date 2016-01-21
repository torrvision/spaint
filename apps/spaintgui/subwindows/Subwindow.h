/**
 * spaintgui: Subwindow.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINTGUI_SUBWINDOW
#define H_SPAINTGUI_SUBWINDOW

#include <ITMLib/Utils/ITMMath.h>

#include "../core/Raycaster.h"

/**
  * \brief An instance of this class can be used to represent a sub-window into
  *        which different types of scene visualisation can be rendered.
  */
class Subwindow
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;

  //#################### PUBLIC VARIABLES ####################
public:
  /** The location of the bottom-right of the subwindow (each component is expressed as a fraction in the range [0,1]). */
  Vector2f m_bottomRight;

  /** An image in which to store the scene visualisation for the sub-window. */
  ITMUChar4Image_Ptr m_image;

  /** The location of the top-left of the subwindow (each component is expressed as a fraction in the range [0,1]). */
  Vector2f m_topLeft;

  /** The type of scene visualisation to render in the sub-window. */
  Raycaster::RaycastType m_type;
};

#endif
