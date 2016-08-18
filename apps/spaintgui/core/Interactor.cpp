/**
 * spaintgui: Interactor.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Interactor.h"
using namespace ITMLib;
using namespace tvginput;

#include <spaint/markers/cpu/VoxelMarker_CPU.h>
#include <spaint/selectiontransformers/SelectionTransformerFactory.h>
#include <spaint/selectors/NullSelector.h>
#include <spaint/selectors/PickingSelector.h>
using namespace spaint;

#ifdef WITH_CUDA
#include <spaint/markers/cuda/VoxelMarker_CUDA.h>
#endif

#ifdef WITH_LEAP
#include <spaint/selectors/LeapSelector.h>
#endif

#ifdef WITH_ARRAYFIRE
#include <spaint/selectors/TouchSelector.h>
#endif

//#################### CONSTRUCTORS ####################

Interactor::Interactor(const Model_Ptr& model)
: m_model(model),
  m_selector(new NullSelector(model->get_settings()))
{
  // Set up the selection transformer.
  const int initialSelectionRadius = 2;
  m_selectionTransformer = SelectionTransformerFactory::make_voxel_to_cube(initialSelectionRadius, model->get_settings()->deviceType);

  // Set up the voxel marker.
  if(model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the CUDA implementation.
    m_voxelMarker.reset(new VoxelMarker_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementation.
    m_voxelMarker.reset(new VoxelMarker_CPU);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Interactor::clear_labels(ClearingSettings settings)
{
  ITMLocalVBA<SpaintVoxel>& localVBA = m_model->get_scene()->localVBA;
  m_voxelMarker->clear_labels(localVBA.GetVoxelBlocks(), localVBA.allocatedSize, settings);
}

Interactor::Selection_CPtr Interactor::get_selection() const
{
  Selection_CPtr selection = m_selector->get_selection();
  return selection && m_selectionTransformer ? Selection_CPtr(m_selectionTransformer->transform_selection(*selection)) : selection;
}

Interactor::SelectionTransformer_CPtr Interactor::get_selection_transformer() const
{
  return m_selectionTransformer;
}

Selector_CPtr Interactor::get_selector() const
{
  return m_selector;
}

const spaint::VoxelMarker_CPtr& Interactor::get_voxel_marker() const
{
  return m_voxelMarker;
}

void Interactor::mark_voxels(const Selection_CPtr& selection, SpaintVoxel::PackedLabel label, const PackedLabels_Ptr& oldLabels, MarkingMode mode)
{
  m_voxelMarker->mark_voxels(*selection, label, m_model->get_scene().get(), oldLabels.get(), mode);
}

void Interactor::mark_voxels(const Selection_CPtr& selection, const PackedLabels_CPtr& labels, MarkingMode mode)
{
  m_voxelMarker->mark_voxels(*selection, *labels, m_model->get_scene().get(), NULL, mode);
}

bool Interactor::selector_is_active() const
{
  return m_selector->is_active();
}

void Interactor::update_selector(const InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono)
{
  // Allow the user to switch between different selectors.
  const Model::Settings_CPtr& settings = m_model->get_settings();
  if(inputState.key_down(KEYCODE_i))
  {
    if(inputState.key_down(KEYCODE_1)) m_selector.reset(new NullSelector(settings));
    else if(inputState.key_down(KEYCODE_2)) m_selector.reset(new PickingSelector(settings));
#ifdef WITH_LEAP
    else if(inputState.key_down(KEYCODE_3)) m_selector.reset(new LeapSelector(settings, m_model->get_scene()));
#endif
#ifdef WITH_ARRAYFIRE
    else if(inputState.key_down(KEYCODE_4))
    {
      const TouchSettings_Ptr touchSettings(new TouchSettings(m_model->get_resources_dir() + "/TouchSettings.xml"));
      const size_t maxKeptTouchPoints = 50;
      m_selector.reset(new TouchSelector(settings, touchSettings, m_model->get_tracking_state(), m_model->get_view(), maxKeptTouchPoints));

      const int initialSelectionRadius = 1;
      m_selectionTransformer = SelectionTransformerFactory::make_voxel_to_cube(initialSelectionRadius, m_model->get_settings()->deviceType);
    }
#endif
  }

  // Update the current selection transformer (if any).
  if(m_selectionTransformer) m_selectionTransformer->update(inputState);

  // Update the current selector.
  m_selector->update(inputState, renderState, renderingInMono);
}
