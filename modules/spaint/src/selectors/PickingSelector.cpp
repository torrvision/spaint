/**
 * spaint: PickingSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/PickingSelector.h"
using namespace tvginput;

#include "picking/cpu/Picker_CPU.h"

#ifdef WITH_CUDA
#include "picking/cuda/Picker_CUDA.h"
#endif

#include "util/MemoryBlockFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

PickingSelector::PickingSelector(const Settings_CPtr& settings)
: Selector(settings),
  m_pickPointFloatMB(MemoryBlockFactory::instance().make_block<Vector3f>(1)),
  m_pickPointShortMB(MemoryBlockFactory::instance().make_block<Vector3s>(1)),
  m_pickPointValid(false)
{
  // Make the picker.
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA)
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
  m_pickPointFloatMB->UpdateHostFromDevice();

  // Convert the pick point from voxel coordinates into scene coordinates and return it.
  float voxelSize = m_settings->sceneParams.voxelSize;
  const Vector3f& pickPoint = *m_pickPointFloatMB->GetData(MEMORYDEVICE_CPU);
  return Eigen::Vector3f(pickPoint.x * voxelSize, pickPoint.y * voxelSize, pickPoint.z * voxelSize);
}

Selector::Selection_CPtr PickingSelector::get_selection() const
{
  return m_pickPointValid ? m_pickPointShortMB : Selection_CPtr();
}

void PickingSelector::update(const InputState& inputState, const RenderState_CPtr& renderState, bool renderingInMono)
{
  // Update whether or not the selector is active.
  m_isActive = renderingInMono && inputState.mouse_button_down(MOUSE_BUTTON_LEFT);

  // If the scene is not being rendered in mono, early out.
  if(!renderingInMono) return;

  // Try and pick an individual voxel.
  m_pickPointValid = false;

  if(!inputState.mouse_position_known()) return;
  int x = inputState.mouse_position_x();
  int y = inputState.mouse_position_y();

  m_pickPointValid = m_picker->pick(x, y, renderState.get(), *m_pickPointFloatMB);
  if(m_pickPointValid) m_picker->to_short(*m_pickPointFloatMB, *m_pickPointShortMB);
}

}
