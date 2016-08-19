/**
 * spaintgui: Raycaster.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "Raycaster.h"

#include <stdexcept>

#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Utils/ITMLibSettings.h>
using namespace ITMLib;
using namespace ORUtils;

#include <spaint/visualisation/cpu/SemanticVisualiser_CPU.h>
#ifdef WITH_CUDA
#include <spaint/visualisation/cuda/SemanticVisualiser_CUDA.h>
#endif
using namespace spaint;

//#################### CONSTRUCTORS ####################

Raycaster::Raycaster(const VisualisationEngine_CPtr& visualisationEngine, const LabelManager_CPtr& labelManager, const Settings_CPtr& settings)
: m_labelManager(labelManager), m_settings(settings), m_visualisationEngine(visualisationEngine)
{
  // Set up the visualisers.
  size_t maxLabelCount = labelManager->get_max_label_count();
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
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

void Raycaster::generate_free_raycast(const ITMUChar4Image_Ptr& output, const Scene_CPtr& scene, const ORUtils::SE3Pose& pose,
                                      const View_CPtr& view, RenderState_Ptr& renderState, RaycastType raycastType,
                                      const boost::optional<Postprocessor>& postprocessor) const
{
  if(!renderState)
  {
    MemoryDeviceType memoryType = m_settings->GetMemoryType();
    renderState.reset(ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(view->depth->noDims, scene->sceneParams, memoryType));
  }

  const ITMIntrinsics *intrinsics = &view->calib->intrinsics_d;
  m_visualisationEngine->FindVisibleBlocks(scene.get(), &pose, intrinsics, renderState.get());
  m_visualisationEngine->CreateExpectedDepths(scene.get(), &pose, intrinsics, renderState.get());

  switch(raycastType)
  {
    case RT_COLOUR:
    {
      m_visualisationEngine->RenderImage(scene.get(), &pose, intrinsics, renderState.get(), renderState->raycastImage,
                                         ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME);
      break;
    }
    case RT_LAMBERTIAN:
    {
      m_visualisationEngine->RenderImage(scene.get(), &pose, intrinsics, renderState.get(), renderState->raycastImage,
                                         ITMLib::IITMVisualisationEngine::RENDER_SHADED_GREYSCALE);
      break;
    }
    case RT_SEMANTICCOLOUR:
    case RT_SEMANTICFLAT:
    case RT_SEMANTICLAMBERTIAN:
    case RT_SEMANTICPHONG:
    {
      const std::vector<Vector3u>& labelColours = m_labelManager->get_label_colours();

      LightingType lightingType = LT_LAMBERTIAN;
      if(raycastType == RT_SEMANTICFLAT) lightingType = LT_FLAT;
      else if(raycastType == RT_SEMANTICPHONG) lightingType = LT_PHONG;

      float labelAlpha = raycastType == RT_SEMANTICCOLOUR ? 0.4f : 1.0f;
      m_visualisationEngine->FindSurface(scene.get(), &pose, intrinsics, renderState.get());
      m_semanticVisualiser->render(scene.get(), &pose, intrinsics, renderState.get(), labelColours, lightingType, labelAlpha, renderState->raycastImage);
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

void Raycaster::get_default_raycast(const ITMUChar4Image_Ptr& output, const RenderState_CPtr& liveRenderState, const boost::optional<Postprocessor>& postprocessor) const
{
  make_postprocessed_cpu_copy(liveRenderState->raycastImage, postprocessor, output);
}

void Raycaster::get_depth_input(const ITMUChar4Image_Ptr& output, const View_CPtr& view) const
{
  prepare_to_copy_visualisation(view->depth->noDims, output);
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
  m_visualisationEngine->DepthToUchar4(output.get(), view->depth);
}

void Raycaster::get_rgb_input(const ITMUChar4Image_Ptr& output, const View_CPtr& view) const
{
  prepare_to_copy_visualisation(view->rgb->noDims, output);
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->rgb->UpdateHostFromDevice();
  output->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void Raycaster::make_postprocessed_cpu_copy(const ITMUChar4Image *inputRaycast, const boost::optional<Postprocessor>& postprocessor,
                                            const ITMUChar4Image_Ptr& outputRaycast) const
{
  // Make sure that the output raycast is of the right size.
  prepare_to_copy_visualisation(inputRaycast->noDims, outputRaycast);

  if(postprocessor)
  {
    // Copy the input raycast to the output raycast on the relevant device (e.g. on the GPU, if that's where the input currently resides).
    outputRaycast->SetFrom(
      inputRaycast,
      m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU
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
      m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU
    );
  }
}

void Raycaster::prepare_to_copy_visualisation(const Vector2i& inputSize, const ITMUChar4Image_Ptr& output) const
{
  output->Clear();
  output->ChangeDims(inputSize);
}
