/**
 * spaintgui: Model.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Model.h"
using namespace ORUtils;
using namespace tvginput;

#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
using namespace ITMLib;

#include <spaint/markers/VoxelMarkerFactory.h>
#include <spaint/selectiontransformers/SelectionTransformerFactory.h>
#include <spaint/selectors/NullSelector.h>
#include <spaint/selectors/PickingSelector.h>
using namespace spaint;

#ifdef WITH_LEAP
#include <spaint/selectors/LeapSelector.h>
#endif

#ifdef WITH_ARRAYFIRE
#include <spaint/selectors/TouchSelector.h>
#endif

//#################### CONSTRUCTORS ####################

Model::Model(const Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize, const TrackingState_Ptr& trackingState,
             const Settings_CPtr& settings, const std::string& resourcesDir, const LabelManager_Ptr& labelManager)
: m_depthImageSize(depthImageSize),
  m_labelManager(labelManager),
  m_resourcesDir(resourcesDir),
  m_rgbImageSize(rgbImageSize),
  m_scene(scene),
  m_selector(new NullSelector(settings)),
  m_semanticLabel(0),
  m_settings(settings),
  m_trackingState(trackingState),
  m_visualisationEngine(ITMVisualisationEngineFactory::MakeVisualisationEngine<SpaintVoxel,ITMVoxelIndex>(settings->deviceType)),
  m_voxelMarker(VoxelMarkerFactory::make_voxel_marker(settings->deviceType))
{
  // Set up the selection transformer.
  const int initialSelectionRadius = 2;
  m_selectionTransformer = SelectionTransformerFactory::make_voxel_to_cube(initialSelectionRadius, settings->deviceType);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Model::clear_labels(ClearingSettings settings)
{
  ITMLocalVBA<SpaintVoxel>& localVBA = m_scene->localVBA;
  m_voxelMarker->clear_labels(localVBA.GetVoxelBlocks(), localVBA.allocatedSize, settings);
}

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

Model::Selection_CPtr Model::get_selection() const
{
  Selection_CPtr selection = m_selector->get_selection();
  return selection && m_selectionTransformer ? Selection_CPtr(m_selectionTransformer->transform_selection(*selection)) : selection;
}

SelectionTransformer_CPtr Model::get_selection_transformer() const
{
  return m_selectionTransformer;
}

Selector_CPtr Model::get_selector() const
{
  return m_selector;
}

SpaintVoxel::Label Model::get_semantic_label() const
{
  return m_semanticLabel;
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

const VoxelMarker_CPtr& Model::get_voxel_marker() const
{
  return m_voxelMarker;
}

void Model::mark_voxels(const Selection_CPtr& selection, SpaintVoxel::PackedLabel label, const PackedLabels_Ptr& oldLabels, MarkingMode mode)
{
  m_voxelMarker->mark_voxels(*selection, label, m_scene.get(), oldLabels.get(), mode);
}

void Model::mark_voxels(const Selection_CPtr& selection, const PackedLabels_CPtr& labels, MarkingMode mode)
{
  m_voxelMarker->mark_voxels(*selection, *labels, m_scene.get(), NULL, mode);
}

bool Model::selector_is_active() const
{
  return m_selector->is_active();
}

void Model::set_semantic_label(SpaintVoxel::Label semanticLabel)
{
  m_semanticLabel = semanticLabel;
}

void Model::set_view(ITMView *view)
{
  if(m_view.get() != view) m_view.reset(view);
}

void Model::update_selector(const InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono)
{
  // Allow the user to switch between different selectors.
  if(inputState.key_down(KEYCODE_i))
  {
    if(inputState.key_down(KEYCODE_1)) m_selector.reset(new NullSelector(m_settings));
    else if(inputState.key_down(KEYCODE_2)) m_selector.reset(new PickingSelector(m_settings));
#ifdef WITH_LEAP
    else if(inputState.key_down(KEYCODE_3)) m_selector.reset(new LeapSelector(settings, m_model->get_scene()));
#endif
#ifdef WITH_ARRAYFIRE
    else if(inputState.key_down(KEYCODE_4))
    {
      const TouchSettings_Ptr touchSettings(new TouchSettings(m_resourcesDir + "/TouchSettings.xml"));
      const size_t maxKeptTouchPoints = 50;
      m_selector.reset(new TouchSelector(m_settings, touchSettings, m_trackingState, m_view, maxKeptTouchPoints));

      const int initialSelectionRadius = 1;
      m_selectionTransformer = SelectionTransformerFactory::make_voxel_to_cube(initialSelectionRadius, m_settings->deviceType);
    }
#endif
  }

  // Update the current selection transformer (if any).
  if(m_selectionTransformer) m_selectionTransformer->update(inputState);

  // Update the current selector.
  m_selector->update(inputState, renderState, renderingInMono);
}
