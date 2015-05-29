#include <iostream>

#ifdef __APPLE__
  // Prevent a warning in OVR_Types.h.
  #undef static_assert
#endif

#include <OVR_CAPI.h>
#include <Extras/OVR_Math.h>
using namespace OVR;

void quit(ovrHmd hmd)
{
  ovrHmd_Destroy(hmd);
  ovr_Shutdown();
  exit(0);
}

int main()
{
  ovr_Initialize();
  ovrHmd hmd = ovrHmd_Create(0);

  if(!hmd)
  {
    std::cout << "Could not find the Oculus!\n";
    quit(hmd);
  }

  // Get the resolution of the head-mounted display.
  ovrSizei resolution = hmd->Resolution;
  std::cout << "HMD Resolution: " << resolution.w << 'x' << resolution.h << '\n';

  // Configure and start the sensor that provides the Rift's pose and motion.
  std::cout << "Configuring tracking...";
  std::cout.flush();
  ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);
  std::cout << "done" << std::endl;

  // Recentre the pose.
  ovrHmd_RecenterPose(hmd);

  for(;;)
  {
    // Query the HMD for the current tracking state.
    ovrTrackingState ts = ovrHmd_GetTrackingState(hmd, ovr_GetTimeInSeconds());

    // If we were able to get the orientation of the HMD, print out the Euler angles.
    if(ts.StatusFlags & ovrStatus_OrientationTracked)
    {
      Posef pose = ts.HeadPose.ThePose;
      float yaw, eyePitch, eyeRoll;
      pose.Rotation.GetEulerAngles<Axis_Y,Axis_X,Axis_Z>(&yaw, &eyePitch, &eyeRoll);

      std::cout << yaw << ' ' << eyePitch << ' ' << eyeRoll << '\n';
    }
  }

  quit(hmd);

  return 0;
}
