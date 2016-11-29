/**
 * spaint: LeapSelector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "selectors/LeapSelector.h"

#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
using namespace ITMLib;
using namespace ORUtils;

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

#include "picking/PickerFactory.h"
#include "selectiontransformers/SelectionTransformerFactory.h"
#include "util/CameraFactory.h"
#include "util/CameraPoseConverter.h"
#include "util/MemoryBlockFactory.h"
using namespace rigging;
using namespace tvginput;

namespace spaint {

//#################### CONSTRUCTORS ####################

LeapSelector::LeapSelector(const Settings_CPtr& settings, const VoxelVisualisationEngine_CPtr& visualisationEngine, Mode mode, const std::string& fiducialID)
: Selector(settings),
  m_camera(CameraFactory::make_default_camera()),
  m_fiducialID(fiducialID),
  m_mode(mode),
  m_picker(PickerFactory::make_picker(settings->deviceType)),
  m_pickPointFloatMB(MemoryBlockFactory::instance().make_block<Vector3f>(1)),
  m_pickPointShortMB(MemoryBlockFactory::instance().make_block<Vector3s>(1)),
  m_visualisationEngine(visualisationEngine)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void LeapSelector::accept(const SelectorVisitor& visitor) const
{
  visitor.visit(*this);
}

const Camera& LeapSelector::get_camera() const
{
  return *m_camera;
}

const Leap::Frame& LeapSelector::get_frame() const
{
  return m_frame;
}

LeapSelector::Mode LeapSelector::get_mode() const
{
  return m_mode;
}

boost::optional<Eigen::Vector3f> LeapSelector::get_position() const
{
  // If the last update did not yield a valid pick point, early out.
  if(!m_pickPointValid) return boost::none;

  // Convert the pick point from voxel coordinates into scene coordinates and return it.
  return Picker::get_positions<Eigen::Vector3f>(*m_pickPointFloatMB, m_settings->sceneParams.voxelSize)[0];
}

Selector::Selection_CPtr LeapSelector::get_selection() const
{
  return m_pickPointValid ? m_pickPointShortMB : Selection_CPtr();
}

void LeapSelector::update(const InputState& inputState, const SLAMState_CPtr& slamState, const VoxelRenderState_CPtr& renderState, bool renderingInMono)
{
  // Update the camera representing the Leap Motion controller's coordinate frame.
  Fiducial_Ptr fiducial = MapUtil::lookup(slamState->get_fiducials(), m_fiducialID, Fiducial_Ptr());
  if(fiducial)
  {
    SimpleCamera c = CameraPoseConverter::pose_to_camera(fiducial->pose());
    m_camera.reset(new SimpleCamera(c.p(), -c.v(), c.n()));
  }

  // Get the current frame of data from the Leap Motion.
  m_frame = m_leap.frame();

  // If the current frame is invalid, or the user is not trying to interact with the scene using a single hand, early out.
  // Note that we do not currently support multi-hand selection, although this may change in the future.
  if(!m_frame.isValid() || m_frame.hands().count() != 1) return;

  // Update whether or not the selector is active.
  m_isActive = inputState.key_down(KEYCODE_l);

  // Find the position of the tip of the index finger in world coordinates.
  const Leap::Finger& indexFinger = m_frame.hands()[0].fingers()[1];
  Eigen::Vector3f fingerPosWorld = from_leap_position(indexFinger.tipPosition());

  switch(m_mode)
  {
    case MODE_POINT:
    {
      // Find the direction of the index finger in world coordinates.
      Eigen::Vector3f fingerDirWorld = from_leap_direction(indexFinger.direction());

      // Generate a raycast of the scene from a camera that points along the index finger.
      VoxelRenderState_Ptr fingerRenderState(ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState(renderState->raycastResult->noDims, &m_settings->sceneParams, m_settings->GetMemoryType()));
      SimpleCamera indexFingerCamera(fingerPosWorld, fingerDirWorld, Eigen::Vector3f(0.0f, -1.0f, 0.0f));
      SE3Pose indexFingerPose = CameraPoseConverter::camera_to_pose(indexFingerCamera);
      m_visualisationEngine->FindSurface(slamState->get_voxel_scene().get(), &indexFingerPose, &slamState->get_intrinsics(), fingerRenderState.get());

      // Use the picker to determine the voxel that was hit (if any).
      m_pickPointValid = m_picker->pick(renderState->raycastResult->noDims.x / 2, renderState->raycastResult->noDims.y / 2, fingerRenderState.get(), *m_pickPointFloatMB);
      if(m_pickPointValid) m_picker->to_short(*m_pickPointFloatMB, *m_pickPointShortMB);

      break;
    }
    case MODE_TOUCH:
    {
      // Convert the position of the tip of the index finger from world coordinates to voxel coordinates.
      Eigen::Vector3f fingerPosVoxels = fingerPosWorld / m_settings->sceneParams.voxelSize;

      // Record the selected voxel.
      *m_pickPointShortMB->GetData(MEMORYDEVICE_CPU) = Vector3f(fingerPosVoxels.x(), fingerPosVoxels.y(), fingerPosVoxels.z()).toShortRound();
      if(m_settings->deviceType == ITMLibSettings::DEVICE_CUDA) m_pickPointShortMB->UpdateDeviceFromHost();

      break;
    }
  }
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Eigen::Vector3f LeapSelector::from_leap_direction(const Leap::Vector& leapDir) const
{
  const Eigen::Vector3f x = -m_camera->u(), y = -m_camera->v(), z = m_camera->n();

  // The Leap coordinate system has x pointing right, y pointing up and z pointing out of the screen, whereas
  // the InfiniTAM coordinate system has x pointing right, y pointing down and z pointing into the screen. As
  // such, we need to flip y and z when converting from the Leap coordinate system to our one.
  return leapDir.x * x - leapDir.y * y - leapDir.z * z;
}

Eigen::Vector3f LeapSelector::from_leap_position(const Leap::Vector& leapPos) const
{
  // The Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the Leap position by 1000.
  Eigen::Vector3f offset = from_leap_direction(leapPos) / 1000.0f;

  return m_camera->p() + offset;
}

float LeapSelector::from_leap_size(float leapSize)
{
  // The Leap measures in millimetres, whereas InfiniTAM measures in metres, so we need to divide the size by 1000.
  return leapSize / 1000.0f;
}

}
