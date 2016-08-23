/**
 * spaintgui: Subwindow.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Subwindow.h"
using namespace spaint;

//#################### CONSTRUCTORS ####################

Subwindow::Subwindow(const Vector2f& topLeft, const Vector2f& bottomRight, const std::string& sceneID, VisualisationGenerator::VisualisationType type, const Vector2i& imgSize)
: m_bottomRight(bottomRight),
  m_image(new ITMUChar4Image(imgSize, true, true)),
  m_sceneID(sceneID),
  m_topLeft(topLeft),
  m_type(type)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2f& Subwindow::bottom_right() const
{
  return m_bottomRight;
}

const ITMUChar4Image_Ptr& Subwindow::get_image()
{
  return m_image;
}

ITMUChar4Image_CPtr Subwindow::get_image() const
{
  return m_image;
}

const std::string& Subwindow::get_scene_id() const
{
  return m_sceneID;
}

VisualisationGenerator::VisualisationType Subwindow::get_type() const
{
  return m_type;
}

float Subwindow::height() const
{
  return m_bottomRight.y - m_topLeft.y;
}

void Subwindow::set_type(VisualisationGenerator::VisualisationType type)
{
  m_type = type;
}

const Vector2f& Subwindow::top_left() const
{
  return m_topLeft;
}

float Subwindow::width() const
{
  return m_bottomRight.x - m_topLeft.x;
}
