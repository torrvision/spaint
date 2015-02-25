/**
 * spaint: PickingSelector.cpp
 */

#include "selectors/PickingSelector.h"

#include "selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"
#ifdef WITH_CUDA
#include "selectiontransformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

PickingSelector::PickingSelector(const Settings_CPtr& settings)
: m_radius(2), m_settings(settings)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<Vector3f> PickingSelector::pick(int x, int y, const RenderState_CPtr& renderState) const
{
  if(!m_raycastResult) m_raycastResult.reset(new ITMFloat4Image(renderState->raycastResult->noDims, true, true));

  // FIXME: It's inefficient to copy the raycast result across from the GPU each time we want to perform a picking operation.
  m_raycastResult->SetFrom(
    renderState->raycastResult,
    m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4f>::CUDA_TO_CPU : ORUtils::MemoryBlock<Vector4f>::CPU_TO_CPU
  );

  const Vector4f *imageData = m_raycastResult->GetData(MEMORYDEVICE_CPU);
  Vector4f voxelData = imageData[y * m_raycastResult->noDims.x + x];
  return voxelData.w > 0 ? boost::optional<Vector3f>(Vector3f(voxelData.x, voxelData.y, voxelData.z)) : boost::none;
}

Selector::Selection_CPtr PickingSelector::select_voxels(const InputState& inputState, const RenderState_CPtr& renderState) const
{
  // Try and get the position of the mouse.
  if(!inputState.mouse_position_known()) return Selection_CPtr();
  int x = inputState.mouse_position_x();
  int y = inputState.mouse_position_y();

  // Try and pick an individual voxel. If we don't hit anything, early out.
  m_pickPoint = pick(x, y, renderState);
  if(!m_pickPoint) return Selection_CPtr();

  // Make the transformer that we need in order to expand the selection to the specified radius.
  boost::shared_ptr<const SelectionTransformer> selectionTransformer;
  const ITMLibSettings::DeviceType deviceType = m_settings->deviceType;
  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    selectionTransformer.reset(new VoxelToCubeSelectionTransformer_CUDA(m_radius));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    selectionTransformer.reset(new VoxelToCubeSelectionTransformer_CPU(m_radius));
  }

  // Make a selection that contains only the voxel we picked and transfer it to the GPU.
  ORUtils::MemoryBlock<Vector3s> pickPointMB(1, true, true);
  pickPointMB.GetData(MEMORYDEVICE_CPU)[0] = m_pickPoint->toShortRound();
  if(deviceType == ITMLibSettings::DEVICE_CUDA) pickPointMB.UpdateDeviceFromHost();

  // Expand it using the transformer.
  MemoryDeviceType memoryDeviceType = deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3s> > cubeMB(new ORUtils::MemoryBlock<Vector3s>(
    selectionTransformer->compute_output_selection_size(pickPointMB),
    memoryDeviceType)
  );
  selectionTransformer->transform_selection(pickPointMB, *cubeMB);

  return cubeMB;
}

void PickingSelector::update(const InputState& inputState)
{
  // Allow the user to change the selection radius.
  static bool canChange = true;
  if(inputState.key_down(SDLK_LEFTBRACKET))
  {
    if(canChange && m_radius > 1) --m_radius;
    canChange = false;
  }
  else if(inputState.key_down(SDLK_RIGHTBRACKET))
  {
    if(canChange && m_radius < 10) ++m_radius;
    canChange = false;
  }
  else canChange = true;
}

}
