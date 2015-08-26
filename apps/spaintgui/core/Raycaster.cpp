/**
 * spaintgui: Raycaster.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Raycaster.h"

#include <stdexcept>

#include <ITMLib/Engine/ITMVisualisationEngine.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp>

#include <spaint/visualisers/cpu/SemanticVisualiser_CPU.h>
#ifdef WITH_CUDA
#include <spaint/visualisers/cuda/SemanticVisualiser_CUDA.h>
#endif
using namespace spaint;

//#################### CONSTRUCTORS ####################

Raycaster::Raycaster(const Model_CPtr& model, const VisualisationEngine_Ptr& visualisationEngine, const RenderState_Ptr& liveRenderState)
: m_liveRenderState(liveRenderState), m_model(model), m_visualisationEngine(visualisationEngine)
{
  // Set up the visualisers.
  size_t maxLabelCount = m_model->get_label_manager()->get_max_label_count();
  if(model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the CUDA implementations.
    m_semanticVisualiser.reset(new SemanticVisualiser_CUDA(maxLabelCount));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementations.
    m_semanticVisualiser.reset(new SemanticVisualiser_CPU(maxLabelCount));
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void Raycaster::generate_free_raycast(const UChar4Image_Ptr& output, RenderState_Ptr& renderState, const ITMPose& pose, RaycastType raycastType,
                                      const boost::optional<Postprocessor>& postprocessor) const
{
  const ITMIntrinsics *intrinsics = &m_model->get_view()->calib->intrinsics_d;
  Model::Scene_CPtr scene = m_model->get_scene();
  Model::View_CPtr view = m_model->get_view();

  if(!renderState) renderState.reset(m_visualisationEngine->CreateRenderState(m_model->get_depth_image_size()));

  m_visualisationEngine->FindVisibleBlocks(&pose, intrinsics, renderState.get());
  m_visualisationEngine->CreateExpectedDepths(&pose, intrinsics, renderState.get());

  switch(raycastType)
  {
    case RT_COLOUR:
    {
      m_visualisationEngine->RenderImage(&pose, intrinsics, renderState.get(), renderState->raycastImage,
                                         ITMLib::Engine::IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME);
      break;
    }
    case RT_LAMBERTIAN:
    {
      m_visualisationEngine->RenderImage(&pose, intrinsics, renderState.get(), renderState->raycastImage,
                                         ITMLib::Engine::IITMVisualisationEngine::RENDER_SHADED_GREYSCALE);
      break;
    }
    case RT_SEMANTICCOLOUR:
    case RT_SEMANTICLAMBERTIAN:
    case RT_SEMANTICPHONG:
    {
      LabelManager_CPtr labelManager = m_model->get_label_manager();
      const std::vector<Vector3u>& labelColours = labelManager->get_label_colours();
      bool usePhong = raycastType == RT_SEMANTICPHONG;
      float labelAlpha = raycastType == RT_SEMANTICCOLOUR ? 0.4f : 1.0f;
      m_visualisationEngine->FindSurface(&pose, intrinsics, renderState.get());
      m_semanticVisualiser->render(scene.get(), &pose, intrinsics, renderState.get(), labelColours, usePhong, labelAlpha, renderState->raycastImage);
      break;
    }
    default:
    {
      // This should never happen.
      throw std::runtime_error("Unknown raycast type");
    }
  }

  make_postprocessed_cpu_copy(renderState->raycastImage, postprocessor, output);
}

void Raycaster::get_default_raycast(const UChar4Image_Ptr& output, const boost::optional<Postprocessor>& postprocessor) const
{
  make_postprocessed_cpu_copy(m_liveRenderState->raycastImage, postprocessor, output);
}

void Raycaster::get_depth_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->depth->noDims, output);
  if(m_model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA) m_model->get_view()->depth->UpdateHostFromDevice();
  m_visualisationEngine->DepthToUchar4(output.get(), m_model->get_view()->depth);
}

const Raycaster::RenderState_Ptr& Raycaster::get_live_render_state()
{
  return m_liveRenderState;
}

void Raycaster::get_rgb_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->rgb->noDims, output);
  if(m_model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA) m_model->get_view()->rgb->UpdateHostFromDevice();
  output->SetFrom(m_model->get_view()->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

const Raycaster::VisualisationEngine_Ptr& Raycaster::get_visualisation_engine()
{
  return m_visualisationEngine;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void Raycaster::make_postprocessed_cpu_copy(const ITMUChar4Image *inputRaycast, const boost::optional<Postprocessor>& postprocessor, const UChar4Image_Ptr& outputRaycast) const
{
  // Make sure that the output raycast is of the right size.
  prepare_to_copy_visualisation(inputRaycast->noDims, outputRaycast);

  const Model::Settings_CPtr& settings = m_model->get_settings();
  if(postprocessor)
  {
    // Copy the input raycast to the output raycast on the relevant device (e.g. on the GPU, if that's where the input currently resides).
    outputRaycast->SetFrom(
      inputRaycast,
      settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU
    );

    // Post-process the output raycast.
    (*postprocessor)(outputRaycast, outputRaycast);

    // Transfer the output raycast to the CPU if necessary (if we're in CPU mode, this is a no-op).
    outputRaycast->UpdateHostFromDevice();
  }
  else
  {
    // If there is no post-processing to be done, copy the input raycast directly into the CPU memory of the output raycast.
    outputRaycast->SetFrom(
      inputRaycast,
      settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU
    );
  }
}

void Raycaster::prepare_to_copy_visualisation(const Vector2i& inputSize, const UChar4Image_Ptr& output) const
{
  output->Clear();
  output->ChangeDims(inputSize);
}
