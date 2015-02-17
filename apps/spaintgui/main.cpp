/**
 * spaintgui: main.cpp
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <Engine/OpenNIEngine.h>
using namespace InfiniTAM::Engine;

#include <SDL.h>

#include <spaint/core/SpaintPipeline.h>
using namespace spaint;

#include "Application.h"

//#################### FUNCTIONS ####################

void quit(const std::string& message, int code = EXIT_FAILURE)
{
  std::cerr << message << '\n';
  SDL_Quit();
  exit(code);
}

int main(int argc, char *argv[])
try
{
  // Initialise SDL.
  if(SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    quit("Error: Failed to initialise SDL.");
  }

  // Parse the command-line arguments.
  if (argc > 4)
  {
    // Note: See the InfiniTAM code for argument details (we use the same arguments here for consistency).
    quit("Usage: spaint [<Calibration Filename> [<OpenNI Device URI> | <RGB Image Mask> <Depth Image Mask>]]");
  }

  std::string calibrationFilename = argc >= 2 ? argv[1] : "./Resources/DefaultCalibration.txt",
              openNIDeviceURI = argc == 3 ? argv[2] : "Default",
              rgbImageMask = argc == 4 ? argv[2] : "",
              depthImageMask = argc == 4 ? argv[3] : "";

  // Specify the InfiniTAM settings.
  boost::shared_ptr<ITMLibSettings> settings(new ITMLibSettings);

  // Specify whether or not to use the Vicon tracker (if it's available).
  bool useVicon = false;

  // If we're using the Vicon tracker, set an appropriate tracking regime for the corresponding ICP tracker.
#ifdef WITH_VICON
  // FIXME: The tracking regime should ultimately be moved out of ITMLibSettings.
  if(useVicon)
  {
    settings->noHierarchyLevels = 2;
    delete [] settings->trackingRegime;
    settings->trackingRegime = new TrackerIterationType[settings->noHierarchyLevels];
    settings->trackingRegime[0] = TRACKER_ITERATION_BOTH;
    settings->trackingRegime[1] = TRACKER_ITERATION_TRANSLATION;
  }
#endif

  // Construct the spaint pipeline.
  SpaintPipeline_Ptr spaintPipeline;
  if(argc == 4)
  {
    std::cout << "[spaint] Reading images from disk: " << rgbImageMask << ' ' << depthImageMask << '\n';
    spaintPipeline.reset(new SpaintPipeline(calibrationFilename, rgbImageMask, depthImageMask, settings));
  }
  else
  {
#ifdef WITH_OPENNI
    std::cout << "[spaint] Reading images from OpenNI device: " << openNIDeviceURI << '\n';
    spaintPipeline.reset(new SpaintPipeline(
      calibrationFilename,
      openNIDeviceURI == "Default" ? boost::none : boost::optional<std::string>(openNIDeviceURI),
      settings,
      useVicon ? "192.168.0.111" : ""
    ));
#else
    quit("Error: OpenNI support not currently available. Reconfigure in CMake with the WITH_OPENNI option set to ON.");
#endif
  }

  // Run the application.
  Application app(spaintPipeline);
  app.run();

  // Shut down SDL.
  SDL_Quit();

  return 0;
}
catch(std::exception& e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
