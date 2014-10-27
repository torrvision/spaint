/**
 * spaintgui: RiftRenderer.cpp
 */

#include "RiftRenderer.h"

#include <stdexcept>

#include <OVR.h>
#include <../Src/Kernel/OVR_Math.h>
using namespace OVR;

//#################### CONSTRUCTORS ####################

RiftRenderer::RiftRenderer()
{
  ovr_Initialize();
  m_hmd = ovrHmd_Create(0);
  if(!m_hmd) throw std::runtime_error("Could not find the Oculus!");

  // Configure and start the sensor that provides the Rift's pose and motion.
  ovrHmd_ConfigureTracking(m_hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);

  // Recentre the pose.
  ovrHmd_RecenterPose(m_hmd);

  // TODO
}

//#################### DESTRUCTOR ####################

RiftRenderer::~RiftRenderer()
{
  ovrHmd_Destroy(m_hmd);
  ovr_Shutdown();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void RiftRenderer::render(const spaint::SpaintEngine_Ptr& spaintEngine) const
{
  //ovrHmd_BeginFrame(m_hmd, ...);

  // Query the HMD for the current tracking state.
  ovrTrackingState trackingState = ovrHmd_GetTrackingState(m_hmd, ovr_GetTimeInSeconds());

  if(trackingState.StatusFlags & ovrStatus_OrientationTracked)
  {
    // Determine the correct InfiniTAM pose from the HMD orientation.
    Posef pose = trackingState.HeadPose.ThePose;
    float yaw, eyePitch, eyeRoll;
    pose.Rotation.GetEulerAngles<Axis_Y,Axis_X,Axis_Z>(&yaw, &eyePitch, &eyeRoll);
    // TODO

    // Render the scene for the Rift.
    // TODO
  }
  else
  {
    // Render an error screen.
    // TODO
  }

  //ovrHmd_EndFrame(m_hmd, ...);
}
