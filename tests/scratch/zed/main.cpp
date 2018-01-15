#include <sl/Camera.hpp>
using namespace sl;

int main(int argc, char **argv)
{
  // Create a ZED camera object
  Camera zed;

  // Set configuration parameters
  InitParameters init_params;
  init_params.sdk_verbose = false; // Disable verbose mode

  // Open the camera
  ERROR_CODE err = zed.open(init_params);
  if (err != SUCCESS)
      exit(-1);

  // Get camera information (ZED serial number)
  int zed_serial = zed.getCameraInformation().serial_number;
  printf("Hello! This is my serial number: %d\n", zed_serial);

  // Close the camera
  zed.close();
  return 0;
}
