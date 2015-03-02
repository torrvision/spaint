/**
 * spaint: PickingSelector.cpp
 */

#include "selectors/PickingSelector.h"

#include "picking/cpu/Picker_CPU.h"
#include "selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"

#ifdef WITH_CUDA
#include "picking/cuda/Picker_CUDA.h"
#include "selectiontransformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

PickingSelector::PickingSelector(const Settings_CPtr& settings)
: m_pickPointFloatMB(1, true, true),
  m_pickPointShortMB(1, true, true),
  m_pickPointValid(false),
  m_radius(2),
  m_settings(settings)
{
  // Make the picker.
  const ITMLibSettings::DeviceType deviceType = m_settings->deviceType;
  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    m_picker.reset(new Picker_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    m_picker.reset(new Picker_CPU);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PickingSelector::accept(const SelectorVisitor& visitor) const
{
  visitor.visit(*this);
}

boost::optional<Eigen::Vector3f> PickingSelector::get_position() const
{
  // If the last update did not yield a valid pick point, early out.
  if(!m_pickPointValid) return boost::none;

  // If the pick point is on the GPU, copy it across to the CPU.
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) m_pickPointFloatMB.UpdateHostFromDevice();

  // Convert the pick point from voxel coordinates into scene coordinates and return it.
  float voxelSize = m_settings->sceneParams.voxelSize;
  const Vector3f& pickPoint = m_pickPointFloatMB.GetData(MEMORYDEVICE_CPU)[0];
  return Eigen::Vector3f(pickPoint.x * voxelSize, pickPoint.y * voxelSize, pickPoint.z * voxelSize);
}

int PickingSelector::get_radius() const
{
  return m_radius;
}

Selector::Selection_CPtr PickingSelector::get_selection() const
{
  // If the last update did not yield a valid pick point, early out.
  if(!m_pickPointValid) return Selection_CPtr();

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

  // Expand the picked point to a cube of voxels using the transformer.
  MemoryDeviceType memoryDeviceType = deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  boost::shared_ptr<ORUtils::MemoryBlock<Vector3s> > cubeMB(new ORUtils::MemoryBlock<Vector3s>(
    selectionTransformer->compute_output_selection_size(m_pickPointShortMB),
    memoryDeviceType
  ));
  selectionTransformer->transform_selection(m_pickPointShortMB, *cubeMB);

  return cubeMB;
}

void PickingSelector::update(const InputState& inputState, const RenderState_CPtr& renderState)
{
  // Update whether or not the selector is active.
  m_isActive = inputState.mouse_button_down(MOUSE_BUTTON_LEFT);

  // Allow the user to change the selection radius.
  const int minRadius = 1;
  const int maxRadius = 10;
  static bool canChange = true;

  if(inputState.key_down(SDLK_LEFTBRACKET))
  {
    if(canChange && m_radius > minRadius) --m_radius;
    canChange = false;
  }
  else if(inputState.key_down(SDLK_RIGHTBRACKET))
  {
    if(canChange && m_radius < maxRadius) ++m_radius;
    canChange = false;
  }
  else canChange = true;

  // Try and pick an individual voxel.
  m_pickPointValid = false;

  if(!inputState.mouse_position_known()) return;
  int x = inputState.mouse_position_x();
  int y = inputState.mouse_position_y();

  m_pickPointValid = m_picker->pick(x, y, renderState.get(), m_pickPointFloatMB);
  if(m_pickPointValid) m_picker->to_short(m_pickPointFloatMB, m_pickPointShortMB);
}

}
