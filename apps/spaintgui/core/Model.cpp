/**
 * spaintgui: Model.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Model.h"
using namespace ORUtils;
using namespace itmx;
using namespace tvginput;

#include <ITMLib/Engines/Visualisation/ITMSurfelVisualisationEngineFactory.h>
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

Model::Model(const Settings_CPtr& settings, const std::string& resourcesDir, size_t maxLabelCount, const MappingServer_Ptr& mappingServer)
: m_labelManager(new LabelManager(maxLabelCount)),
  m_mappingServer(mappingServer),
  m_resourcesDir(resourcesDir),
  m_selector(new NullSelector(settings)),
  m_semanticLabel(0),
  m_settings(settings),
  m_surfelVisualisationEngine(ITMSurfelVisualisationEngineFactory<SpaintSurfel>::make_surfel_visualisation_engine(settings->deviceType)),
  m_voxelMarker(VoxelMarkerFactory::make_voxel_marker(settings->deviceType)),
  m_voxelVisualisationEngine(ITMVisualisationEngineFactory::MakeVisualisationEngine<SpaintVoxel,ITMVoxelIndex>(settings->deviceType))
{
  // Set up the selection transformer.
  const int initialSelectionRadius = 2;
  m_selectionTransformer = SelectionTransformerFactory::make_voxel_to_cube(initialSelectionRadius, settings->deviceType);

  // Set up the visualisation generator.
  m_visualisationGenerator.reset(new VisualisationGenerator(settings, m_labelManager, m_voxelVisualisationEngine, m_surfelVisualisationEngine));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Model::clear_labels(const std::string& sceneID, ClearingSettings settings)
{
  ITMLocalVBA<SpaintVoxel>& localVBA = get_slam_state(sceneID)->get_voxel_scene()->localVBA;
  m_voxelMarker->clear_labels(localVBA.GetVoxelBlocks(), localVBA.allocatedSize, settings);
}

const LabelManager_Ptr& Model::get_label_manager()
{
  return m_labelManager;
}

LabelManager_CPtr Model::get_label_manager() const
{
  return m_labelManager;
}

const MappingServer_Ptr& Model::get_mapping_server()
{
  return m_mappingServer;
}

const std::string& Model::get_resources_dir() const
{
  return m_resourcesDir;
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

const Settings_CPtr& Model::get_settings() const
{
  return m_settings;
}

Model::SurfelVisualisationEngine_CPtr Model::get_surfel_visualisation_engine() const
{
  return m_surfelVisualisationEngine;
}

VisualisationGenerator_CPtr Model::get_visualisation_generator() const
{
  return m_visualisationGenerator;
}

Model::VoxelVisualisationEngine_CPtr Model::get_voxel_visualisation_engine() const
{
  return m_voxelVisualisationEngine;
}

void Model::mark_voxels(const std::string& sceneID, const Selection_CPtr& selection, SpaintVoxel::PackedLabel label,
                        MarkingMode mode, const PackedLabels_Ptr& oldLabels)
{
  m_voxelMarker->mark_voxels(*selection, label, get_slam_state(sceneID)->get_voxel_scene().get(), mode, oldLabels.get());
}

void Model::mark_voxels(const std::string& sceneID, const Selection_CPtr& selection, const PackedLabels_CPtr& labels, MarkingMode mode)
{
  m_voxelMarker->mark_voxels(*selection, *labels, get_slam_state(sceneID)->get_voxel_scene().get(), mode);
}

void Model::set_leap_fiducial_id(const std::string& leapFiducialID)
{
  m_leapFiducialID = leapFiducialID;
}

void Model::set_semantic_label(SpaintVoxel::Label semanticLabel)
{
  m_semanticLabel = semanticLabel;
}

void Model::update_selector(const InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono)
{
  // Allow the user to switch between different selectors.
  if(inputState.key_down(KEYCODE_i))
  {
    if(inputState.key_down(KEYCODE_1)) m_selector.reset(new NullSelector(m_settings));
    else if(inputState.key_down(KEYCODE_2)) m_selector.reset(new PickingSelector(m_settings));
#ifdef WITH_LEAP
    else if(inputState.key_down(KEYCODE_3))
    {
      m_selector.reset(new LeapSelector(m_settings, m_voxelVisualisationEngine, LeapSelector::MODE_POINT, m_leapFiducialID));
    }
#endif
#ifdef WITH_ARRAYFIRE
    else if(inputState.key_down(KEYCODE_4))
    {
      const TouchSettings_Ptr touchSettings(new TouchSettings(m_resourcesDir + "/TouchSettings.xml"));
      const size_t maxKeptTouchPoints = 50;
      m_selector.reset(new TouchSelector(m_settings, touchSettings, get_slam_state(Model::get_world_scene_id())->get_depth_image_size(), maxKeptTouchPoints));

      const int initialSelectionRadius = 1;
      m_selectionTransformer = SelectionTransformerFactory::make_voxel_to_cube(initialSelectionRadius, m_settings->deviceType);
    }
#endif
  }

  // Update the current selection transformer (if any).
  if(m_selectionTransformer) m_selectionTransformer->update(inputState);

  // Update the current selector.
  m_selector->update(inputState, slamState, renderState, renderingInMono);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::string Model::get_world_scene_id()
{
  return "World";
}

//#################### DISAMBIGUATORS ####################

RefiningRelocaliser_CPtr Model::get_relocaliser(const std::string& sceneID) const
{
  return SLAMContext::get_relocaliser(sceneID);
}

std::vector<std::string> Model::get_scene_ids() const
{
  return SLAMContext::get_scene_ids();
}

const SLAMState_Ptr& Model::get_slam_state(const std::string& sceneID)
{
  return SLAMContext::get_slam_state(sceneID);
}

SLAMState_CPtr Model::get_slam_state(const std::string& sceneID) const
{
  return SLAMContext::get_slam_state(sceneID);
}
