/**
 * spaint: LeapSelector.cpp
 */

#include "selectors/LeapSelector.h"

#include "selectiontransformers/cpu/VoxelToCubeSelectionTransformer_CPU.h"

#ifdef WITH_CUDA
#include "selectiontransformers/cuda/VoxelToCubeSelectionTransformer_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

LeapSelector::LeapSelector(const Settings_CPtr& settings, const Scene_CPtr& scene)
: Selector(settings),
  m_pickPointShortMB(1, true, true),
  m_scene(scene)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LeapSelector::accept(const SelectorVisitor& visitor) const
{
  visitor.visit(*this);
}

const Leap::Frame& LeapSelector::get_frame() const
{
  return m_frame;
}

Selector::Selection_CPtr LeapSelector::get_selection() const
{
#if 0
  // TEMPORARY
  Selection_Ptr selection(new Selection(1, true, true));
  selection->SetFrom(
    &m_pickPointShortMB,
    m_settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector3s>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector3s>::CPU_TO_CPU
  );
  return selection;
#else
  // Make the transformer that we need in order to expand the selection to the specified radius.
  boost::shared_ptr<const SelectionTransformer> selectionTransformer;
  const ITMLibSettings::DeviceType deviceType = m_settings->deviceType;
  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    selectionTransformer.reset(new VoxelToCubeSelectionTransformer_CUDA(10));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    selectionTransformer.reset(new VoxelToCubeSelectionTransformer_CPU(10));
  }

  // Expand the picked point to a cube of voxels using the transformer.
  MemoryDeviceType memoryDeviceType = deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  Selection_Ptr cubeMB(new Selection(selectionTransformer->compute_output_selection_size(m_pickPointShortMB), memoryDeviceType));
  selectionTransformer->transform_selection(m_pickPointShortMB, *cubeMB);

  return cubeMB;
#endif
}

void LeapSelector::update(const InputState& inputState, const RenderState_CPtr& renderState)
{
  m_frame = m_leap.frame();
  if(!m_frame.isValid() || m_frame.hands().count() != 1) return;

  // Update whether or not the selector is active.
  m_isActive = true;

  // Find the position of the tip of the index finger in world coordinates.
  Leap::Vector fingerPosLM = m_frame.hands()[0].fingers()[1].tipPosition();
  Eigen::Vector3f fingerPosWorld = from_leap_vector(fingerPosLM);

  // Convert this world coordinate position into voxel coordinates.
  Eigen::Vector3f fingerPosVoxels = fingerPosWorld / m_scene->sceneParams->voxelSize;

  // Record the selected voxel.
  *m_pickPointShortMB.GetData(MEMORYDEVICE_CPU) = Vector3f(fingerPosVoxels.x(), fingerPosVoxels.y(), fingerPosVoxels.z()).toShortRound();
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) m_pickPointShortMB.UpdateDeviceFromHost();
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Eigen::Vector3f LeapSelector::from_leap_vector(const Leap::Vector& leapVec)
{
  // The Leap coordinate system has x pointing right, y pointing up and z pointing out of the screen, whereas
  // the InfiniTAM coordinate system has x pointing right, y pointing down and z pointing into the screen. As
  // such, we need to flip y and z when converting from the Leap coordinate system to our one. Moreover, the
  // Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the result by 1000.
  return Eigen::Vector3f(leapVec.x, -(leapVec.y - 225), -leapVec.z) / 1000;
}

}
