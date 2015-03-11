/**
 * spaint: RiftTracker.cpp
 */

#include "trackers/RiftTracker.h"

#include <iostream>
#include <stdexcept>

namespace spaint {

//#################### CONSTRUCTORS ####################

RiftTracker::RiftTracker()
{
  // Get a handle to the Rift.
  m_hmd = ovrHmd_Create(0);
  if(!m_hmd) throw std::runtime_error("[spaint] Could not find the Rift!");

  // Configure tracking on the Rift.
  ovrHmd_ConfigureTracking(m_hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);
  ovrHmd_RecenterPose(m_hmd);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void RiftTracker::SetInitialPose(ITMTrackingState *trackingState)
{
  for(;;)
  {
    ovrTrackingState riftTrackingState = ovrHmd_GetTrackingState(m_hmd, ovr_GetTimeInSeconds());
    if(riftTrackingState.StatusFlags & ovrStatus_OrientationTracked)
    {
      trackingState->pose_d->SetR(extract_rotation_matrix(riftTrackingState));
      trackingState->pose_d->Coerce();
      break;
    }
  }
}

void RiftTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  ovrTrackingState riftTrackingState = ovrHmd_GetTrackingState(m_hmd, ovr_GetTimeInSeconds());
  if(riftTrackingState.StatusFlags & ovrStatus_OrientationTracked)
  {
    trackingState->pose_d->SetR(extract_rotation_matrix(riftTrackingState));
    trackingState->pose_d->Coerce();
  }
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

Matrix3f RiftTracker::extract_rotation_matrix(const ovrTrackingState& riftTrackingState)
{
  Matrix3f R;

  OVR::Posef pose = riftTrackingState.HeadPose.ThePose;
  OVR::Matrix4<float> r(pose.Rotation);

  // TODO: Comment this.
  R(0,0) = r.M[0][0]; R(1,0) = r.M[0][1];  R(2,0) = r.M[0][2];
  R(0,1) = r.M[1][0]; R(1,1) = r.M[1][1];  R(2,1) = -r.M[1][2];
  R(0,2) = r.M[2][0]; R(1,2) = -r.M[2][1]; R(2,2) = r.M[2][2];

  return R;
}

}
