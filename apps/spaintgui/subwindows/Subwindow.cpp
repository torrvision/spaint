/**
 * spaintgui: Subwindow.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Subwindow.h"

//#################### CONSTRUCTORS ####################

Subwindow::Subwindow(const Vector2f& topLeft, const Vector2f& bottomRight, Raycaster::RaycastType type, const Vector2i& imgSize)
: m_bottomRight(bottomRight),
  m_image(new ITMUChar4Image(imgSize, true, true)),
  m_topLeft(topLeft),
  m_type(type)
{}
