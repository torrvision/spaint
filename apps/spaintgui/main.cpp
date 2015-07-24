/**
 * spaintgui: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <cstdlib>
#include <iostream>
#include <string>

// Note: This must appear before anything that could include SDL.h, since it includes boost/asio.hpp, a header that has a WinSock conflict with SDL.h.
#include "Application.h"

#ifdef WITH_ARRAYFIRE
  #include <arrayfire.h>
#endif

#include <Engine/OpenNIEngine.h>
using namespace InfiniTAM::Engine;

#ifdef WITH_OVR
  #include <OVR_CAPI.h>
#endif

#include <spaint/core/SpaintPipeline.h>
#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

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

#ifdef WITH_ARRAYFIRE
  // Choose a device for ArrayFire.
  if(af::getDeviceCount() > 1)
  {
    af::setDevice(1);
  }
#endif

#ifdef WITH_OVR
  // If we built with Rift support, initialise the Rift SDK.
  ovr_Initialize();
#endif

  // Parse the command-line arguments.
  if (argc > 4)
  {
    // Note: See the InfiniTAM code for argument details (we use the same arguments here for consistency).
    quit("Usage: spaint [<Calibration Filename> [<OpenNI Device URI> | <RGB Image Mask> <Depth Image Mask>]]");
  }

  std::string calibrationFilename = argc >= 2 ? argv[1] : "",
              openNIDeviceURI = argc == 3 ? argv[2] : "Default",
              rgbImageMask = argc == 4 ? argv[2] : "",
              depthImageMask = argc == 4 ? argv[3] : "";

  // Specify the settings.
  boost::shared_ptr<ITMLibSettings> settings(new ITMLibSettings);
  SpaintPipeline::TrackerType trackerType = SpaintPipeline::TRACKER_INFINITAM;
  std::string trackerParams;

  // If we're trying to use the Rift tracker:
  if(trackerType == SpaintPipeline::TRACKER_RIFT)
  {
#ifdef WITH_OVR
    // If the Rift isn't available when the program runs, make sure that we're not trying to use the Rift tracker.
    if(ovrHmd_Detect() == 0) trackerType = SpaintPipeline::TRACKER_INFINITAM;
#else
    // If we haven't built with Rift support, make sure that we're not trying to use the Rift tracker.
    trackerType = SpaintPipeline::TRACKER_INFINITAM;
#endif
  }

  // If we're trying to use the Vicon tracker:
  if(trackerType == SpaintPipeline::TRACKER_VICON)
  {
#ifdef WITH_VICON
    // If we built with Vicon support, specify the Vicon host (at present this refers to Iain's machine on the
    // oculab network in the JR), and set an appropriate tracking regime for the corresponding ICP tracker.
    trackerParams = "192.168.0.111:801";

    // FIXME: The tracking regime should ultimately be moved out of ITMLibSettings.
    settings->noHierarchyLevels = 2;
    delete [] settings->trackingRegime;
    settings->trackingRegime = new TrackerIterationType[settings->noHierarchyLevels];
    settings->trackingRegime[0] = TRACKER_ITERATION_BOTH;
    settings->trackingRegime[1] = TRACKER_ITERATION_TRANSLATION;
#else
    // If we haven't built with Vicon support, make sure that we're not trying to use the Vicon tracker.
    trackerType = SpaintPipeline::TRACKER_INFINITAM;
#endif
  }

  // Pass the device type to the memory block factory.
  MemoryBlockFactory::instance().set_device_type(settings->deviceType);

  // Construct the spaint pipeline.
  SpaintPipeline_Ptr spaintPipeline;
  std::string resourcesDir = Application::resources_dir().string();
  if(argc == 4)
  {
    std::cout << "[spaint] Reading images from disk: " << rgbImageMask << ' ' << depthImageMask << '\n';
    spaintPipeline.reset(new SpaintPipeline(calibrationFilename, rgbImageMask, depthImageMask, settings, resourcesDir));
  }
  else
  {
#ifdef WITH_OPENNI
    std::cout << "[spaint] Reading images from OpenNI device: " << openNIDeviceURI << '\n';
    boost::optional<std::string> uri = openNIDeviceURI == "Default" ? boost::none : boost::optional<std::string>(openNIDeviceURI);
    bool useInternalCalibration = true;
    spaintPipeline.reset(new SpaintPipeline(calibrationFilename, uri, settings, resourcesDir, trackerType, trackerParams, useInternalCalibration));
#else
    quit("Error: OpenNI support not currently available. Reconfigure in CMake with the WITH_OPENNI option set to ON.");
#endif
  }

  // Run the application.
  Application app(spaintPipeline);
  app.run();

#ifdef WITH_OVR
  // If we built with Rift support, shut down the Rift SDK.
  ovr_Shutdown();
#endif

  // Shut down SDL.
  SDL_Quit();

  return 0;
}
catch(std::exception& e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
