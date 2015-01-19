/**
 * spaint: SpaintRaycaster.cpp
 */

#include "core/SpaintRaycaster.h"

#include <stdexcept>

#include <ITMLib/Engine/ITMTrackerFactory.h>
#include <ITMLib/Engine/ITMVisualisationEngine.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp>

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintRaycaster::SpaintRaycaster(const SpaintModel_CPtr& model)
: m_model(model)
{
  // Set up the InfiniTAM visualisation engine.
  if(model->get_settings().deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the GPU implementation of the visualisation engine.
    m_visualisationEngine.reset(new ITMVisualisationEngine_CUDA<SpaintVoxel,ITMVoxelIndex>);
#else
    // This should never happen as things stand - we set useGPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementation of the visualisation engine.
    m_visualisationEngine.reset(new ITMVisualisationEngine_CPU<SpaintVoxel,ITMVoxelIndex>);
  }

  // Set up the live render state.
  m_liveRenderState.reset(m_visualisationEngine->CreateRenderState(
    model->get_scene().get(),
    ITMTrackerFactory::GetTrackedImageSize(model->get_settings(), model->get_rgb_image_size(), model->get_depth_image_size())
  ));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SpaintRaycaster::generate_free_raycast(const UChar4Image_Ptr& output, RenderState_Ptr& renderState, const ITMPose& pose) const
{
  SpaintModel::Scene_CPtr scene = m_model->get_scene();
  SpaintModel::View_CPtr view = m_model->get_view();
  ITMLibSettings settings = m_model->get_settings();

  if(!renderState) renderState.reset(m_visualisationEngine->CreateRenderState(scene.get(), output->noDims));

  m_visualisationEngine->FindVisibleBlocks(scene.get(), &pose, &view->calib->intrinsics_d, renderState.get());
  m_visualisationEngine->CreateExpectedDepths(scene.get(), &pose, &view->calib->intrinsics_d, renderState.get());
  m_visualisationEngine->RenderImage(scene.get(), &pose, &view->calib->intrinsics_d, renderState.get(), renderState->raycastImage, false);

  if(settings.deviceType == ITMLibSettings::DEVICE_CUDA) renderState->raycastImage->UpdateHostFromDevice();
  output->SetFrom(renderState->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

void SpaintRaycaster::get_default_raycast(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_liveRenderState->raycastImage, output);
  output->SetFrom(m_liveRenderState->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

void SpaintRaycaster::get_depth_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->depth, output);
  m_visualisationEngine->DepthToUchar4(output.get(), m_model->get_view()->depth);
}

const SpaintRaycaster::RenderState_Ptr& SpaintRaycaster::get_live_render_state()
{
  return m_liveRenderState;
}

void SpaintRaycaster::get_rgb_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->rgb, output);
  output->SetFrom(m_model->get_view()->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

const SpaintRaycaster::VisualisationEngine_Ptr& SpaintRaycaster::get_visualisation_engine()
{
  return m_visualisationEngine;
}

}
