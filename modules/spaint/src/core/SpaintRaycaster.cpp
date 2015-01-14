/**
 * spaint: SpaintRaycaster.cpp
 */

#include "core/SpaintRaycaster.h"

#include <ITMLib/Engine/ITMVisualisationEngine.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp>

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintRaycaster::SpaintRaycaster(const SpaintModel_CPtr& model, const ITMLibSettings& settings)
: m_model(model), m_settings(settings)
{
  // Set up the InfiniTAM visualisation engine.
  if(settings.useGPU)
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
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SpaintRaycaster::generate_free_raycast(const UChar4Image_Ptr& output, VisualisationState_Ptr& visualisationState, const ITMPose& pose) const
{
  if(!visualisationState) visualisationState.reset(m_visualisationEngine->allocateInternalState(output->noDims));

  SpaintModel::Scene_CPtr scene = m_model->get_scene();
  SpaintModel::View_CPtr view = m_model->get_view();
  m_visualisationEngine->FindVisibleBlocks(scene.get(), &pose, &view->calib->intrinsics_d, visualisationState.get());
  m_visualisationEngine->CreateExpectedDepths(scene.get(), &pose, &view->calib->intrinsics_d, visualisationState->minmaxImage, visualisationState.get());
  m_visualisationEngine->RenderImage(scene.get(), &pose, &view->calib->intrinsics_d, visualisationState.get(), visualisationState->outputImage, false);

  if(m_settings.useGPU) visualisationState->outputImage->UpdateHostFromDevice();
  output->SetFrom(visualisationState->outputImage);
}

void SpaintRaycaster::get_default_raycast(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_tracking_state()->rendering, output);
  output->SetFrom(m_model->get_tracking_state()->rendering);
}

void SpaintRaycaster::get_depth_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->depth, output);
  m_visualisationEngine->DepthToUchar4(output.get(), m_model->get_view()->depth);
}

void SpaintRaycaster::get_rgb_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->rgb, output);
  output->SetFrom(m_model->get_view()->rgb);
}

const SpaintRaycaster::VisualisationEngine_Ptr& SpaintRaycaster::get_visualisation_engine()
{
  return m_visualisationEngine;
}

}
