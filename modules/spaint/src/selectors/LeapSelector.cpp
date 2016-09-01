/**
 * spaint: LeapSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/LeapSelector.h"
using namespace ITMLib;
using namespace tvginput;

#include "selectiontransformers/SelectionTransformerFactory.h"
#include "util/MemoryBlockFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

LeapSelector::LeapSelector(const Settings_CPtr& settings)
: Selector(settings),
  m_pickPointShortMB(MemoryBlockFactory::instance().make_block<Vector3s>(1))
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
  return m_pickPointShortMB;
}

void LeapSelector::update(const InputState& inputState, const VoxelRenderState_CPtr& renderState, bool renderingInMono)
{
  // Get the current frame of data from the Leap Motion.
  m_frame = m_leap.frame();

  // If the current frame is invalid, or the user is not trying to interact with the scene using a single hand, early out.
  // Note that we do not currently support multi-hand selection, although this may change in the future.
  if(!m_frame.isValid() || m_frame.hands().count() != 1) return;

  // Update whether or not the selector is active.
  m_isActive = true;

  // Find the position of the tip of the index finger in world coordinates.
  Leap::Vector fingerPosLM = m_frame.hands()[0].fingers()[1].tipPosition();
  Eigen::Vector3f fingerPosWorld = from_leap_vector(fingerPosLM);

  // Convert this world coordinate position into voxel coordinates.
  Eigen::Vector3f fingerPosVoxels = fingerPosWorld / m_settings->sceneParams.voxelSize;

  // Record the selected voxel.
  *m_pickPointShortMB->GetData(MEMORYDEVICE_CPU) = Vector3f(fingerPosVoxels.x(), fingerPosVoxels.y(), fingerPosVoxels.z()).toShortRound();
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) m_pickPointShortMB->UpdateDeviceFromHost();
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

float LeapSelector::from_leap_size(float leapSize)
{
  // The Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the size by 1000.
  return leapSize / 1000.0f;
}

Eigen::Vector3f LeapSelector::from_leap_vector(const Leap::Vector& leapVec)
{
  // FIXME: This is currently a quick hack - I'm specifying that the camera origin is 30cm above the Leap (i.e. the camera should initially be positioned just above the Leap).
  Eigen::Vector3f offset(0.0f, 300.0f, 0.0f);

  // The Leap coordinate system has x pointing right, y pointing up and z pointing out of the screen, whereas
  // the InfiniTAM coordinate system has x pointing right, y pointing down and z pointing into the screen. As
  // such, we need to flip y and z when converting from the Leap coordinate system to our one. Moreover, the
  // Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the result by 1000.
  return (Eigen::Vector3f(leapVec.x, -leapVec.y, -leapVec.z) + offset) / 1000.0f;
}

}
