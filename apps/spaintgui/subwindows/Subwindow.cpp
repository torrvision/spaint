/**
 * spaintgui: Subwindow.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Subwindow.h"
using namespace rigging;
using namespace spaint;

//#################### CONSTRUCTORS ####################

Subwindow::Subwindow(const Vector2f& topLeft, const Vector2f& bottomRight, const std::string& sceneID, VisualisationGenerator::VisualisationType type, const Vector2i& imgSize)
: m_bottomRight(bottomRight),
  m_cameraMode(CM_FOLLOW),
  m_image(new ITMUChar4Image(imgSize, true, true)),
  m_sceneID(sceneID),
  m_surfelFlag(false),
  m_topLeft(topLeft),
  m_type(type)
{
  reset_camera();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2f& Subwindow::bottom_right() const
{
  return m_bottomRight;
}

const CompositeCamera_Ptr& Subwindow::get_camera() const
{
  return m_camera;
}

Subwindow::CameraMode Subwindow::get_camera_mode() const
{
  return m_cameraMode;
}

const ITMUChar4Image_Ptr& Subwindow::get_image()
{
  return m_image;
}

ITMUChar4Image_CPtr Subwindow::get_image() const
{
  return m_image;
}

Subwindow::RenderState_Ptr& Subwindow::get_render_state(int viewIndex)
{
  return m_renderStates[viewIndex];
}

Subwindow::RenderState_CPtr Subwindow::get_render_state(int viewIndex) const
{
  return m_renderStates[viewIndex];
}

const std::string& Subwindow::get_scene_id() const
{
  return m_sceneID;
}

bool Subwindow::get_surfel_flag() const
{
  return m_surfelFlag;
}

VisualisationGenerator::VisualisationType Subwindow::get_type() const
{
  return m_type;
}

float Subwindow::height() const
{
  return m_bottomRight.y - m_topLeft.y;
}

void Subwindow::reset_camera()
{
  m_camera.reset(new CompositeCamera(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, -1.0f, 0.0f)));
}

void Subwindow::set_camera_mode(CameraMode cameraMode)
{
  m_cameraMode = cameraMode;
}

void Subwindow::set_surfel_flag(bool surfelFlag)
{
  m_surfelFlag = surfelFlag;
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
