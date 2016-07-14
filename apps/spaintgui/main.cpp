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

#include <InputSource/OpenNIEngine.h>
using namespace InputSource;
using namespace ITMLib;

#ifdef WITH_GLUT
  #include <spaint/ogl/WrappedGL.h>
  #include <GL/glut.h>
  #undef WIN32_LEAN_AND_MEAN
#endif

#ifdef WITH_OVR
  #include <OVR_CAPI.h>
#endif

#include <spaint/util/ColourConversion_Shared.h>
#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

#include "core/Pipeline.h"

//#################### FUNCTIONS ####################

void quit(const std::string& message, int code = EXIT_FAILURE)
{
  std::cerr << message << '\n';
  SDL_Quit();
  exit(code);
}

/**
 * \brief Converts an RGB colour to YCrCb.
 *
 * \param rgb The RGB colour.
 * \return    The resulting of converting the colour to YCrCb.
 */
inline Vector3u convert_rgb_to_ycbcr(const Vector3u& rgb)
{
  // See "A Survey on Pixel-Based Skin Color Detection Techniques" by Vezhnevets et al., equation (9).
  float r = rgb.r / 255.0f;
  float g = rgb.g / 255.0f;
  float b = rgb.b / 255.0f;

  float y = 0.299f * r + 0.587f * g + 0.114f * b;
  //float cr = 0.5f * r - 0.419f * g - 0.081f * b;
  //float cb = -0.169f * r - 0.331f * g + 0.5f * b;
  float cr = r - y;
  float cb = b - y;

  /*return Vector3u(
    static_cast<unsigned char>(y * 255.0f + 0.5f),
    static_cast<unsigned char>(cr * 255.0f + 0.5f),
    static_cast<unsigned char>(cb * 255.0f + 0.5f)
  );*/
  return Vector3u(
    (unsigned char)CLAMP(ROUND(0.299f * rgb.r + 0.587f * rgb.g + 0.114f * rgb.b), 0, 255),
    (unsigned char)CLAMP(ROUND(128 - 0.169f * rgb.r - 0.331f * rgb.g + 0.5f * rgb.b), 0, 255),
    (unsigned char)CLAMP(ROUND(128 + 0.5f * rgb.r - 0.419f * rgb.g - 0.081f * rgb.b), 0, 255)
  );
}

inline Vector3u convert_ycbcr_to_rgb(const Vector3u& ycrcb)
{
  int Y = ycrcb.x;
  int Cbm = ycrcb.y - 128;
  int Crm = ycrcb.z - 128;
  return Vector3u(
    (unsigned char)CLAMP(ROUND(Y + 1.4f * Crm), 0, 255),
    (unsigned char)CLAMP(ROUND(Y - 0.343f * Cbm - 0.711f * Crm), 0, 255),
    (unsigned char)CLAMP(ROUND(Y + 1.765f * Cbm), 0, 255)
  );
}

void output_colour(const Vector3u& c)
{
  Vector3i ci(c.r, c.g, c.b);
  std::cout << ci << '\n';
}

int main(int argc, char *argv[])
try
{
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(0,0,0))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(127,0,0))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(255,0,0))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(0,255,0))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(0,0,255))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(255,255,0))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(255,0,255))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(0,255,255))));
  output_colour(convert_ycbcr_to_rgb(convert_rgb_to_ycbcr(Vector3u(255,255,255))));
  return 0;

  // Initialise SDL.
  if(SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    quit("Error: Failed to initialise SDL.");
  }

#ifdef WITH_GLUT
  // Initialise GLUT (used for text rendering only).
  glutInit(&argc, argv);
#endif

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
  settings->trackerConfig = "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

  Pipeline::TrackerType trackerType = Pipeline::TRACKER_INFINITAM;
  std::string trackerParams;

  // If we're trying to use the Rift tracker:
  if(trackerType == Pipeline::TRACKER_RIFT)
  {
#ifdef WITH_OVR
    // If the Rift isn't available when the program runs, make sure that we're not trying to use the Rift tracker.
    if(ovrHmd_Detect() == 0) trackerType = Pipeline::TRACKER_INFINITAM;
#else
    // If we haven't built with Rift support, make sure that we're not trying to use the Rift tracker.
    trackerType = Pipeline::TRACKER_INFINITAM;
#endif
  }

  // If we're trying to use the Vicon tracker:
  if(trackerType == Pipeline::TRACKER_VICON || trackerType == Pipeline::TRACKER_ROBUSTVICON)
  {
#ifdef WITH_VICON
    // If we built with Vicon support, specify the Vicon host (at present this refers to Iain's machine on the
    // oculab network in the JR), and set an appropriate tracking regime for the corresponding ICP tracker.
    trackerParams = "192.168.10.1:801";

#if 0
    // FIXME: The tracking regime should ultimately be moved out of ITMLibSettings.
    settings->noHierarchyLevels = 2;
    delete [] settings->trackingRegime;
    settings->trackingRegime = new TrackerIterationType[settings->noHierarchyLevels];
    settings->trackingRegime[0] = TRACKER_ITERATION_BOTH;
    settings->trackingRegime[1] = TRACKER_ITERATION_TRANSLATION;
#endif
#else
    // If we haven't built with Vicon support, make sure that we're not trying to use the Vicon tracker.
    trackerType = Pipeline::TRACKER_INFINITAM;
#endif
  }

  // Pass the device type to the memory block factory.
  MemoryBlockFactory::instance().set_device_type(settings->deviceType);

  // Construct the pipeline.
  Pipeline_Ptr pipeline;
  std::string resourcesDir = Application::resources_dir().string();
  if(argc == 4)
  {
    std::cout << "[spaint] Reading images from disk: " << rgbImageMask << ' ' << depthImageMask << '\n';
    pipeline.reset(new Pipeline(calibrationFilename, rgbImageMask, depthImageMask, settings, resourcesDir));
  }
  else
  {
#ifdef WITH_OPENNI
    std::cout << "[spaint] Reading images from OpenNI device: " << openNIDeviceURI << '\n';
    boost::optional<std::string> uri = openNIDeviceURI == "Default" ? boost::none : boost::optional<std::string>(openNIDeviceURI);
    bool useInternalCalibration = !uri; // if reading from a file, assume that the provided calibration is to be used
    pipeline.reset(new Pipeline(calibrationFilename, uri, settings, resourcesDir, trackerType, trackerParams, useInternalCalibration));
#else
    quit("Error: OpenNI support not currently available. Reconfigure in CMake with the WITH_OPENNI option set to ON.");
#endif
  }

  // Run the application.
  Application app(pipeline);
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
