#include <iostream>

#include <OVR.h>

int main()
{
  ovr_Initialize();
  ovrHmd hmd = ovrHmd_Create(0);

  if(hmd)
  {
    // Get the resolution of the head-mounted display.
    ovrSizei resolution = hmd->Resolution;

    std::cout << resolution.w << 'x' << resolution.h << '\n';
  }
  else std::cout << "Could not find the Oculus!\n";

  ovrHmd_Destroy(hmd);
  ovr_Shutdown();

  return 0;
}
