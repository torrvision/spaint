/**
 * spaint: SpaintRaycaster.cpp
 */

#include "core/SpaintRaycaster.h"

#include <stdexcept>

#include <ITMLib/Engine/ITMVisualisationEngine.cpp>
#include <ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp>

#include "selection/transformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"
#include "visualisers/cpu/SemanticVisualiser_CPU.h"

#ifdef WITH_CUDA
#include "selection/transformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h"
#include "visualisers/cuda/SemanticVisualiser_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintRaycaster::SpaintRaycaster(const SpaintModel_CPtr& model, const VisualisationEngine_Ptr& visualisationEngine, const RenderState_Ptr& liveRenderState)
: m_liveRenderState(liveRenderState), m_model(model), m_visualisationEngine(visualisationEngine)
{
  // Set up the visualisers.
  if(model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the CUDA implementations.
    m_semanticVisualiser.reset(new SemanticVisualiser_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementations.
    m_semanticVisualiser.reset(new SemanticVisualiser_CPU);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SpaintRaycaster::generate_free_raycast(const UChar4Image_Ptr& output, RenderState_Ptr& renderState, const ITMPose& pose, RaycastType raycastType) const
{
  const ITMIntrinsics *intrinsics = &m_model->get_view()->calib->intrinsics_d;
  SpaintModel::Scene_CPtr scene = m_model->get_scene();
  const SpaintModel::Settings_CPtr& settings = m_model->get_settings();
  SpaintModel::View_CPtr view = m_model->get_view();

  if(!renderState) renderState.reset(m_visualisationEngine->CreateRenderState(m_model->get_depth_image_size()));

  m_visualisationEngine->FindVisibleBlocks(&pose, intrinsics, renderState.get());
  m_visualisationEngine->CreateExpectedDepths(&pose, intrinsics, renderState.get());

  switch(raycastType)
  {
    case RT_COLOUR:
    {
      m_visualisationEngine->RenderImage(&pose, intrinsics, renderState.get(), renderState->raycastImage, true);
      break;
    }
    case RT_LAMBERTIAN:
    {
      m_visualisationEngine->RenderImage(&pose, intrinsics, renderState.get(), renderState->raycastImage, false);
      break;
    }
    case RT_SEMANTIC:
    {
      m_visualisationEngine->FindSurface(&pose, intrinsics, renderState.get());
      m_semanticVisualiser->render(scene.get(), &pose, intrinsics, renderState.get(), renderState->raycastImage);
      break;
    }
    default:
    {
      // This should never happen.
      throw std::runtime_error("Unknown raycast type");
    }
  }

  prepare_to_copy_visualisation(renderState->raycastImage->noDims, output);
  output->SetFrom(
    renderState->raycastImage,
    settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU
  );
}

void SpaintRaycaster::get_default_raycast(const UChar4Image_Ptr& output) const
{
  const SpaintModel::Settings_CPtr& settings = m_model->get_settings();
  prepare_to_copy_visualisation(m_liveRenderState->raycastImage->noDims, output);
  output->SetFrom(
    m_liveRenderState->raycastImage,
    settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU
  );
}

void SpaintRaycaster::get_depth_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->depth->noDims, output);
  if(m_model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA) m_model->get_view()->depth->UpdateHostFromDevice();
  m_visualisationEngine->DepthToUchar4(output.get(), m_model->get_view()->depth);
}

const SpaintRaycaster::RenderState_Ptr& SpaintRaycaster::get_live_render_state()
{
  return m_liveRenderState;
}

void SpaintRaycaster::get_rgb_input(const UChar4Image_Ptr& output) const
{
  prepare_to_copy_visualisation(m_model->get_view()->rgb->noDims, output);
  if(m_model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA) m_model->get_view()->rgb->UpdateHostFromDevice();
  output->SetFrom(m_model->get_view()->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
}

const SpaintRaycaster::VisualisationEngine_Ptr& SpaintRaycaster::get_visualisation_engine()
{
  return m_visualisationEngine;
}

boost::optional<Vector3f> SpaintRaycaster::pick(int x, int y, RenderState_CPtr renderState) const
{
  if(!renderState) renderState = m_liveRenderState;
  if(!m_raycastResult) m_raycastResult.reset(new ITMFloat4Image(renderState->raycastResult->noDims, true, true));

  // FIXME: It's inefficient to copy the raycast result across from the GPU each time we want to perform a picking operation.
  m_raycastResult->SetFrom(
    renderState->raycastResult,
    m_model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CPU : ORUtils::MemoryBlock<Vector4f>::CPU_TO_CPU
  );

  const Vector4f *imageData = m_raycastResult->GetData(MEMORYDEVICE_CPU);
  Vector4f voxelData = imageData[y * m_raycastResult->noDims.x + x];
  return voxelData.w > 0 ? boost::optional<Vector3f>(Vector3f(voxelData.x, voxelData.y, voxelData.z)) : boost::none;
}

boost::shared_ptr<ORUtils::MemoryBlock<Vector3s> > SpaintRaycaster::pick_cube(int x, int y, int radius, RenderState_CPtr renderState) const
{
  // Try and pick an individual voxel. If we don't hit anything, early out.
  boost::optional<Vector3f> loc = pick(x, y, renderState);
  if(!loc) return boost::shared_ptr<ORUtils::MemoryBlock<Vector3s> >();

  // Make the transformer that we need in order to expand the selection to the specified radius.
  spaint::SelectionTransformer_CPtr selectionTransformer;
  const ITMLibSettings::DeviceType deviceType = m_model->get_settings()->deviceType;
  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    selectionTransformer.reset(new spaint::VoxelToCubeSelectionTransformer_CUDA(radius));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    selectionTransformer.reset(new spaint::VoxelToCubeSelectionTransformer_CPU(radius));
  }

  // Make a selection that contains only the voxel we picked.
  ORUtils::MemoryBlock<Vector3s> locMB(1, true, true);
  locMB.GetData(MEMORYDEVICE_CPU)[0] = loc->toShortRound();
  locMB.UpdateDeviceFromHost();

  // Expand it using the transformer.
  MemoryDeviceType memoryDeviceType = deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3s> > cubeMB(new ORUtils::MemoryBlock<Vector3s>(selectionTransformer->compute_output_selection_size(locMB), memoryDeviceType));
  selectionTransformer->transform_selection(locMB, *cubeMB);

  return cubeMB;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SpaintRaycaster::prepare_to_copy_visualisation(const Vector2i& inputSize, const UChar4Image_Ptr& output) const
{
  output->Clear();
  output->ChangeDims(inputSize);
}

}
