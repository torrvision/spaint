/**
 * spaintgui: Model.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Model.h"
using namespace spaint;

#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
using namespace ITMLib;
using namespace ORUtils;

//#################### CONSTRUCTORS ####################

Model::Model(const Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize, const TrackingState_Ptr& trackingState,
             const Settings_CPtr& settings, const std::string& resourcesDir, const LabelManager_Ptr& labelManager)
: m_depthImageSize(depthImageSize),
  m_labelManager(labelManager),
  m_resourcesDir(resourcesDir),
  m_rgbImageSize(rgbImageSize),
  m_scene(scene),
  m_settings(settings),
  m_trackingState(trackingState)
{
  // Set up the visualisation engine.
  m_visualisationEngine.reset(ITMVisualisationEngineFactory::MakeVisualisationEngine<SpaintVoxel,ITMVoxelIndex>(settings->deviceType));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& Model::get_depth_image_size() const
{
  return m_depthImageSize;
}

const ITMIntrinsics& Model::get_intrinsics() const
{
  return m_view->calib->intrinsics_d;
}

const LabelManager_Ptr& Model::get_label_manager()
{
  return m_labelManager;
}

LabelManager_CPtr Model::get_label_manager() const
{
  return m_labelManager;
}

const SE3Pose& Model::get_pose() const
{
  return *m_trackingState->pose_d;
}

const std::string& Model::get_resources_dir() const
{
  return m_resourcesDir;
}

const Vector2i& Model::get_rgb_image_size() const
{
  return m_rgbImageSize;
}

const Model::Scene_Ptr& Model::get_scene()
{
  return m_scene;
}

Model::Scene_CPtr Model::get_scene() const
{
  return m_scene;
}

const Model::Settings_CPtr& Model::get_settings() const
{
  return m_settings;
}

const Model::TrackingState_Ptr& Model::get_tracking_state()
{
  return m_trackingState;
}

Model::TrackingState_CPtr Model::get_tracking_state() const
{
  return m_trackingState;
}

const Model::View_Ptr& Model::get_view()
{
  return m_view;
}

Model::View_CPtr Model::get_view() const
{
  return m_view;
}

Model::VisualisationEngine_CPtr Model::get_visualisation_engine() const
{
  return m_visualisationEngine;
}

void Model::set_view(ITMView *view)
{
  if(m_view.get() != view) m_view.reset(view);
}
