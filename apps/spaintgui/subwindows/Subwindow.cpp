/**
 * spaintgui: Subwindow.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Subwindow.h"
using namespace rigging;

#include <spaint/util/CameraFactory.h>
using namespace spaint;

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

//#################### CONSTRUCTORS ####################

Subwindow::Subwindow(const Vector2f& topLeft, const Vector2f& bottomRight, const std::string& sceneID, VisualisationGenerator::VisualisationType type, const Vector2i& imgSize)
: m_allScenesFlag(false),
  m_bottomRight(bottomRight),
  m_cameraMode(CM_FOLLOW),
  m_originalImgSize(imgSize),
  m_remoteFlag(false),
  m_sceneID(sceneID),
  m_surfelFlag(false),
  m_topLeft(topLeft),
  m_type(type)
{
  resize_image(imgSize);
  reset_camera();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2f& Subwindow::bottom_right() const
{
  return m_bottomRight;
}

bool Subwindow::get_all_scenes_flag() const
{
  return m_allScenesFlag;
}

const CompositeCamera_Ptr& Subwindow::get_camera() const
{
  return m_camera;
}

Subwindow::CameraMode Subwindow::get_camera_mode() const
{
  return m_cameraMode;
}

const Eigen::Vector3f& Subwindow::get_camera_up_vector() const
{
  return m_cameraUpVector;
}

const ITMUChar4Image_Ptr& Subwindow::get_image()
{
  return m_image;
}

ITMUChar4Image_CPtr Subwindow::get_image() const
{
  return m_image;
}

const Vector2i& Subwindow::get_original_image_size() const
{
  return m_originalImgSize;
}

bool Subwindow::get_remote_flag() const
{
  return m_remoteFlag;
}

const std::string& Subwindow::get_scene_id() const
{
  return m_sceneID;
}

bool Subwindow::get_surfel_flag() const
{
  return m_surfelFlag;
}

SurfelRenderState_Ptr& Subwindow::get_surfel_render_state(int viewIndex)
{
  return m_surfelRenderStates[viewIndex];
}

SurfelRenderState_CPtr Subwindow::get_surfel_render_state(int viewIndex) const
{
  return MapUtil::lookup(m_surfelRenderStates, viewIndex, SurfelRenderState_Ptr());
}

VisualisationGenerator::VisualisationType Subwindow::get_type() const
{
  return m_type;
}

VoxelRenderState_Ptr& Subwindow::get_voxel_render_state(int viewIndex)
{
  return m_voxelRenderStates[viewIndex];
}

VoxelRenderState_CPtr Subwindow::get_voxel_render_state(int viewIndex) const
{
  return MapUtil::lookup(m_voxelRenderStates, viewIndex, VoxelRenderState_Ptr());
}

float Subwindow::height() const
{
  return m_bottomRight.y - m_topLeft.y;
}

void Subwindow::reset_camera()
{
  m_camera = CameraFactory::make_default_camera<CompositeCamera>();
  m_cameraUpVector = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
}

void Subwindow::resize_image(const Vector2i& newImgSize)
{
  m_image.reset(new ITMUChar4Image(newImgSize, true, true));
  m_voxelRenderStates.clear();
  m_surfelRenderStates.clear();
}

void Subwindow::set_all_scenes_flag(bool allScenesFlag)
{
  m_allScenesFlag = allScenesFlag;
}

void Subwindow::set_camera_mode(CameraMode cameraMode)
{
  m_cameraMode = cameraMode;
}

void Subwindow::set_camera_up_vector(const Eigen::Vector3f& cameraUpVector)
{
  m_cameraUpVector = cameraUpVector;
}

void Subwindow::set_remote_flag(bool remoteFlag)
{
  m_remoteFlag = remoteFlag;
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
