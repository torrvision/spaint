/**
 * spaint: LeapSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/LeapSelector.h"

#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
using namespace ITMLib;
using namespace ORUtils;

#include "picking/PickerFactory.h"
#include "selectiontransformers/SelectionTransformerFactory.h"
#include "util/CameraPoseConverter.h"
#include "util/ImagePersister.h"
#include "util/MemoryBlockFactory.h"
using namespace rigging;
using namespace tvginput;

#include <tvgutil/filesystem/SequentialPathGenerator.h>
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

LeapSelector::LeapSelector(const Settings_CPtr& settings, const VoxelVisualisationEngine_CPtr& visualisationEngine, const VisualisationGenerator_CPtr& visualisationGenerator)
: Selector(settings),
  m_picker(PickerFactory::make_picker(settings->deviceType)),
  m_pickPointFloatMB(MemoryBlockFactory::instance().make_block<Vector3f>(1)),
  m_pickPointShortMB(MemoryBlockFactory::instance().make_block<Vector3s>(1)),
  m_visualisationEngine(visualisationEngine),
  m_visualisationGenerator(visualisationGenerator)
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

boost::optional<Eigen::Vector3f> LeapSelector::get_position() const
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

Selector::Selection_CPtr LeapSelector::get_selection() const
{
  return m_pickPointValid ? m_pickPointShortMB : Selection_CPtr();
}

void LeapSelector::update(const InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono)
{
  // Get the current frame of data from the Leap Motion.
  m_frame = m_leap.frame();

  // If the current frame is invalid, or the user is not trying to interact with the scene using a single hand, early out.
  // Note that we do not currently support multi-hand selection, although this may change in the future.
  if(!m_frame.isValid() || m_frame.hands().count() != 1) return;

  // Update whether or not the selector is active.
  m_isActive = true;

  // Find the position of the tip of the index finger in world coordinates.
  const Leap::Finger& indexFinger = m_frame.hands()[0].fingers()[1];
  Eigen::Vector3f fingerPosWorld = from_leap_position(indexFinger.tipPosition());

#if 1
    // Find the direction of the index finger in world coordinates.
  Eigen::Vector3f fingerDirWorld = from_leap_direction(indexFinger.direction());

  // Generate a raycast of the scene from a camera that points along the index finger.
  VoxelRenderState_Ptr fingerRenderState(ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(renderState->raycastResult->noDims, &m_settings->sceneParams, m_settings->GetMemoryType()));
  SimpleCamera indexFingerCamera(fingerPosWorld, fingerDirWorld, Eigen::Vector3f(0.0f, -1.0f, 0.0f));
  SE3Pose indexFingerPose = CameraPoseConverter::camera_to_pose(indexFingerCamera);
  m_visualisationEngine->FindSurface(slamState->get_voxel_scene().get(), &indexFingerPose, &slamState->get_intrinsics(), fingerRenderState.get());

#if 0
  ITMUChar4Image_Ptr temp(new ITMUChar4Image(renderState->raycastResult->noDims, true, true));
  m_visualisationGenerator->generate_voxel_visualisation(temp, slamState->get_voxel_scene(), indexFingerPose, slamState->get_view(), fingerRenderState, VisualisationGenerator::VT_SCENE_SEMANTICLAMBERTIAN);
  static SequentialPathGenerator gen("C:/wibble/");
  ImagePersister::save_image_on_thread(temp, gen.make_path("%06i.png"));
  gen.increment_index();
#endif
  //*slamState->get_tracking_state()->pose_d = indexFingerPose;

  // Use the picker to determine the voxel that was hit (if any).
  // FIXME: Note that this code is similar to that in PickingSelector - factor out the commonality before merging.

  // Try to pick an individual voxel.
  m_pickPointValid = false;

  int x = 320;
  int y = 240;

  m_pickPointValid = m_picker->pick(x, y, fingerRenderState.get(), *m_pickPointFloatMB);
  if(m_pickPointValid) m_picker->to_short(*m_pickPointFloatMB, *m_pickPointShortMB);
#else
  // Convert this world coordinate position into voxel coordinates.
  Eigen::Vector3f fingerPosVoxels = fingerPosWorld / m_settings->sceneParams.voxelSize;

  // Record the selected voxel.
  *m_pickPointShortMB->GetData(MEMORYDEVICE_CPU) = Vector3f(fingerPosVoxels.x(), fingerPosVoxels.y(), fingerPosVoxels.z()).toShortRound();
  if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) m_pickPointShortMB->UpdateDeviceFromHost();
#endif
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Eigen::Vector3f LeapSelector::from_leap_direction(const Leap::Vector& leapDir)
{
  // The Leap coordinate system has x pointing right, y pointing up and z pointing out of the screen, whereas
  // the InfiniTAM coordinate system has x pointing right, y pointing down and z pointing into the screen. As
  // such, we need to flip y and z when converting from the Leap coordinate system to our one.
  return Eigen::Vector3f(leapDir.x, -leapDir.y, -leapDir.z);
}

Eigen::Vector3f LeapSelector::from_leap_position(const Leap::Vector& leapPos)
{
  // FIXME: This is currently a quick hack - I'm specifying that the camera origin is 30cm above the Leap
  //        (i.e. the camera should initially be positioned just above the Leap).
  Eigen::Vector3f offset(0.0f, 300.0f, 0.0f);

  // The Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the result by 1000.
  return (from_leap_direction(leapPos) + offset) / 1000.0f;
}

float LeapSelector::from_leap_size(float leapSize)
{
  // The Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the size by 1000.
  return leapSize / 1000.0f;
}

}
