/**
 * spaintgui: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

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

#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

#include "core/MultiScenePipeline.h"

//#################### TYPES ####################

struct CommandLineArguments
{
  std::string calibrationFilename;
  bool cameraAfterDisk;
  std::string depthImageMask;
  int initialFrameNumber;
  bool noRelocaliser;
  std::string openNIDeviceURI;
  std::string rgbImageMask;
  std::string sequenceName;
  std::string sequenceType;
};

//#################### FUNCTIONS ####################

bool parse_command_line(int argc, char *argv[], CommandLineArguments& args)
{
  // Specify the possible options.
  po::options_description genericOptions("Generic options");
  genericOptions.add_options()
    ("help", "produce help message")
    ("calib,c", po::value<std::string>(&args.calibrationFilename)->default_value(""), "calibration filename")
    ("cameraAfterDisk", po::bool_switch(&args.cameraAfterDisk), "switch to the camera after a disk sequence")
    ("noRelocaliser", po::bool_switch(&args.noRelocaliser), "don't use the relocaliser")
  ;

  po::options_description cameraOptions("Camera options");
  cameraOptions.add_options()
    ("uri,u", po::value<std::string>(&args.openNIDeviceURI)->default_value("Default"), "OpenNI device URI")
  ;

  po::options_description diskSequenceOptions("Disk sequence options");
  diskSequenceOptions.add_options()
    ("depthMask,d", po::value<std::string>(&args.depthImageMask)->default_value(""), "depth image mask")
    ("initialFrame,n", po::value<int>(&args.initialFrameNumber)->default_value(0), "initial frame number")
    ("rgbMask,r", po::value<std::string>(&args.rgbImageMask)->default_value(""), "RGB image mask")
    ("sequenceName,s", po::value<std::string>(&args.sequenceName)->default_value(""), "sequence name")
    ("sequenceType", po::value<std::string>(&args.sequenceType)->default_value("sequence"), "sequence type")
  ;

  po::options_description options;
  options.add(genericOptions);
  options.add(cameraOptions);
  options.add(diskSequenceOptions);

  // Actually parse the command line.
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, options), vm);
  po::notify(vm);

  // If the user specifies the --help flag, print a help message.
  if(vm.count("help"))
  {
    std::cout << options << '\n';
    return false;
  }

  // If the user specifies a sequence name, set the depth / RGB image masks and the calibration filename appropriately.
  if(args.sequenceName != "")
  {
    boost::filesystem::path dir = find_subdir_from_executable(args.sequenceType + "s") / args.sequenceName;

    args.depthImageMask = (dir / "depthm%06i.pgm").string();
    args.rgbImageMask = (dir / "rgbm%06i.ppm").string();

    // If the user hasn't explicitly specified a calibration file, try to find one in the sequence directory.
    if(args.calibrationFilename == "")
    {
      boost::filesystem::path defaultCalibrationFilename = dir / "calib.txt";
      if(boost::filesystem::exists(defaultCalibrationFilename))
      {
        args.calibrationFilename = defaultCalibrationFilename.string();
      }
    }
  }

  return true;
}

void quit(const std::string& message, int code = EXIT_FAILURE)
{
  std::cerr << message << '\n';
  SDL_Quit();
  exit(code);
}

int main(int argc, char *argv[])
try
{
  // Parse the command-line arguments.
  CommandLineArguments args;
  if(!parse_command_line(argc, argv, args))
  {
    return 0;
  }

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

  // Specify the settings.
  boost::shared_ptr<ITMLibSettings> settings(new ITMLibSettings);
  if(args.cameraAfterDisk || !args.noRelocaliser) settings->behaviourOnFailure = ITMLibSettings::FAILUREMODE_RELOCALISE;
  settings->trackerConfig = "type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

  TrackerType trackerType = TRACKER_INFINITAM;
  std::string trackerParams;

  // If we're trying to use the Rift tracker:
  if(trackerType == TRACKER_RIFT)
  {
#ifdef WITH_OVR
    // If the Rift isn't available when the program runs, make sure that we're not trying to use the Rift tracker.
    if(ovrHmd_Detect() == 0) trackerType = TRACKER_INFINITAM;
#else
    // If we haven't built with Rift support, make sure that we're not trying to use the Rift tracker.
    trackerType = TRACKER_INFINITAM;
#endif
  }

  // If we're trying to use the Vicon tracker:
  if(trackerType == TRACKER_VICON || trackerType == TRACKER_ROBUSTVICON)
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
    trackerType = TRACKER_INFINITAM;
#endif
  }

  // Pass the device type to the memory block factory.
  MemoryBlockFactory::instance().set_device_type(settings->deviceType);

  // Construct the image source engine.
  boost::shared_ptr<CompositeImageSourceEngine> imageSourceEngine(new CompositeImageSourceEngine);

  if(args.depthImageMask != "")
  {
    std::cout << "[spaint] Reading images from disk: " << args.rgbImageMask << ' ' << args.depthImageMask << '\n';
    ImageMaskPathGenerator pathGenerator(args.rgbImageMask.c_str(), args.depthImageMask.c_str());
    imageSourceEngine->addSubengine(new ImageFileReader<ImageMaskPathGenerator>(args.calibrationFilename.c_str(), pathGenerator, args.initialFrameNumber));
  }

  if(args.depthImageMask == "" || args.cameraAfterDisk)
  {
#ifdef WITH_OPENNI
    std::cout << "[spaint] Reading images from OpenNI device: " << args.openNIDeviceURI << '\n';
    boost::optional<std::string> uri = args.openNIDeviceURI == "Default" ? boost::none : boost::optional<std::string>(args.openNIDeviceURI);
    bool useInternalCalibration = !uri; // if reading from a file, assume that the provided calibration is to be used
    imageSourceEngine->addSubengine(new OpenNIEngine(args.calibrationFilename.c_str(), uri ? uri->c_str() : NULL, useInternalCalibration
#if USE_LOW_USB_BANDWIDTH_MODE
      // If there is insufficient USB bandwidth available to support 640x480 RGB input, use 320x240 instead.
      , Vector2i(320, 240)
#endif
    ));
#else
    quit("Error: OpenNI support not currently available. Reconfigure in CMake with the WITH_OPENNI option set to ON.");
#endif
  }

  // Construct the label manager.
  const size_t maxLabelCount = 10;
  LabelManager_Ptr labelManager(new LabelManager(maxLabelCount));

  // Construct the multi-scene pipeline.
  const unsigned int seed = 12345;
  MultiScenePipeline_Ptr pipeline(new MultiScenePipeline(settings, Application::resources_dir().string(), labelManager));
  pipeline->add_single_scene_pipeline("World", imageSourceEngine, seed, trackerType, trackerParams);

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
