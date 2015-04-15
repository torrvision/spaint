/**
 * spaint: SpaintInteractor.cpp
 */

#include "core/SpaintInteractor.h"

#include "markers/cpu/VoxelMarker_CPU.h"
#include "selectiontransformers/SelectionTransformerFactory.h"
#include "selectors/NullSelector.h"
#include "selectors/PickingSelector.h"

#ifdef WITH_CUDA
#include "markers/cuda/VoxelMarker_CUDA.h"
#endif

#ifdef WITH_LEAP
#include "selectors/LeapSelector.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintInteractor::SpaintInteractor(const SpaintModel_Ptr& model)
: m_model(model),
  m_selector(new NullSelector(model->get_settings())),
  m_semanticLabel(0)
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

SpaintInteractor::Selection_CPtr SpaintInteractor::get_selection() const
{
  Selection_CPtr selection = m_selector->get_selection();
  return selection && m_selectionTransformer ? Selection_CPtr(m_selectionTransformer->transform_selection(*selection)) : selection;
}

SpaintInteractor::SelectionTransformer_CPtr SpaintInteractor::get_selection_transformer() const
{
  return m_selectionTransformer;
}

Selector_CPtr SpaintInteractor::get_selector() const
{
  return m_selector;
}

SpaintVoxel::LabelType SpaintInteractor::get_semantic_label() const
{
  return m_semanticLabel;
}

void SpaintInteractor::mark_voxels(const Selection_CPtr& selection, SpaintVoxel::LabelType label)
{
  m_voxelMarker->mark_voxels(*selection, label, m_model->get_scene().get());
}

bool SpaintInteractor::selector_is_active() const
{
  return m_selector->is_active();
}

void SpaintInteractor::set_semantic_label(SpaintVoxel::LabelType semanticLabel)
{
  m_semanticLabel = semanticLabel;
}

void SpaintInteractor::update_selector(const InputState& inputState, const RenderState_CPtr& renderState)
{
  // Allow the user to switch between different selectors.
  const SpaintModel::Settings_CPtr& settings = m_model->get_settings();
  if(inputState.key_down(SDLK_i))
  {
    if(inputState.key_down(SDLK_1)) m_selector.reset(new NullSelector(settings));
    else if(inputState.key_down(SDLK_2)) m_selector.reset(new PickingSelector(settings));
#ifdef WITH_LEAP
    else if(inputState.key_down(SDLK_3)) m_selector.reset(new LeapSelector(settings, m_model->get_scene()));
#endif
  }

  // Update the current selection transformer (if any).
  if(m_selectionTransformer) m_selectionTransformer->update(inputState);

  // Update the current selector.
  m_selector->update(inputState, renderState);
}

}
