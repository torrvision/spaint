/**
 * spaint: VisualisationGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "visualisation/VisualisationGenerator.h"

#include <stdexcept>

#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Utils/ITMLibSettings.h>
using namespace ITMLib;
using namespace ORUtils;

#include "visualisation/VisualiserFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VisualisationGenerator::VisualisationGenerator(const VisualisationEngine_CPtr& visualisationEngine, const LabelManager_CPtr& labelManager,
                                               const Settings_CPtr& settings)
: m_labelManager(labelManager),
  m_semanticVisualiser(VisualiserFactory::make_semantic_visualiser(labelManager->get_max_label_count(), settings->deviceType)),
  m_settings(settings),
  m_visualisationEngine(visualisationEngine)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VisualisationGenerator::generate_free_raycast(const ITMUChar4Image_Ptr& output, const SpaintScene_CPtr& scene, const ORUtils::SE3Pose& pose,
                                                   const View_CPtr& view, RenderState_Ptr& renderState, VisualisationType visualisationType,
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

  switch(visualisationType)
  {
    case VT_SCENE_COLOUR:
    {
      m_visualisationEngine->RenderImage(scene.get(), &pose, intrinsics, renderState.get(), renderState->raycastImage,
                                         ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME);
      break;
    }
    case VT_SCENE_NORMAL:
    {
      m_visualisationEngine->RenderImage(scene.get(), &pose, intrinsics, renderState.get(), renderState->raycastImage,
                                         ITMLib::IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL);
      break;
    }
    case VT_SCENE_SEMANTICCOLOUR:
    case VT_SCENE_SEMANTICFLAT:
    case VT_SCENE_SEMANTICLAMBERTIAN:
    case VT_SCENE_SEMANTICPHONG:
    {
      const std::vector<Vector3u>& labelColours = m_labelManager->get_label_colours();

      LightingType lightingType = LT_LAMBERTIAN;
      if(visualisationType == VT_SCENE_SEMANTICFLAT) lightingType = LT_FLAT;
      else if(visualisationType == VT_SCENE_SEMANTICPHONG) lightingType = LT_PHONG;

      float labelAlpha = visualisationType == VT_SCENE_SEMANTICCOLOUR ? 0.4f : 1.0f;
      m_visualisationEngine->FindSurface(scene.get(), &pose, intrinsics, renderState.get());
      m_semanticVisualiser->render(scene.get(), &pose, intrinsics, renderState.get(), labelColours, lightingType, labelAlpha, renderState->raycastImage);
      break;
    }
    case VT_SCENE_LAMBERTIAN:
    default:
    {
      m_visualisationEngine->RenderImage(scene.get(), &pose, intrinsics, renderState.get(), renderState->raycastImage,
                                         ITMLib::IITMVisualisationEngine::RENDER_SHADED_GREYSCALE);
      break;
    }
  }

  make_postprocessed_cpu_copy(renderState->raycastImage, postprocessor, output);
}

void VisualisationGenerator::get_default_raycast(const ITMUChar4Image_Ptr& output, const RenderState_CPtr& liveRenderState, const boost::optional<Postprocessor>& postprocessor) const
{
  make_postprocessed_cpu_copy(liveRenderState->raycastImage, postprocessor, output);
}

void VisualisationGenerator::get_depth_input(const ITMUChar4Image_Ptr& output, const View_CPtr& view) const
{
  prepare_to_copy_visualisation(view->depth->noDims, output);
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
  m_visualisationEngine->DepthToUchar4(output.get(), view->depth);
}

void VisualisationGenerator::get_rgb_input(const ITMUChar4Image_Ptr& output, const View_CPtr& view) const
{
  prepare_to_copy_visualisation(view->rgb->noDims, output);
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->rgb->UpdateHostFromDevice();
  output->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VisualisationGenerator::make_postprocessed_cpu_copy(const ITMUChar4Image *inputRaycast, const boost::optional<Postprocessor>& postprocessor,
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

void VisualisationGenerator::prepare_to_copy_visualisation(const Vector2i& inputSize, const ITMUChar4Image_Ptr& output) const
{
  output->Clear();
  output->ChangeDims(inputSize);
}

}
