/**
 * spaintgui: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>

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

#ifdef WITH_OPENCV
  #include <spaint/fiducials/ArUcoFiducialDetector.h>
#endif

#include <spaint/imagesources/AsyncImageSourceEngine.h>
#include <spaint/util/MemoryBlockFactory.h>
using namespace spaint;

#include <tvgutil/containers/ParametersContainer.h>
#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

#include "core/SemanticPipeline.h"
#include "core/SLAMPipeline.h"

//#################### NAMESPACE ALIASES ####################

namespace bf = boost::filesystem;
namespace po = boost::program_options;

//#################### TYPES ####################

struct CommandLineArguments
{
  bool batch;
  std::string calibrationFilename;
  bool cameraAfterDisk;
  std::vector<std::string> depthImageMask;
  bool detectFiducials;
  std::string experimentTag;
  int initialFrameNumber;
  bool mapSurfels;
  bool noRelocaliser;
  bool noTracker;
  std::string openNIDeviceURI;
  std::string pipelineType;
  std::string poseFilesMask;
  bool poseFromDisk;
  size_t prefetchBufferCapacity;
  bool renderFiducials;
  std::vector<std::string> rgbImageMask;
  bool saveMeshOnExit;
  std::vector<std::string> sequenceSpecifier;
  std::string sequenceType;
  bool trackSurfels;
};

//#################### FUNCTIONS ####################

bool parse_command_line(int argc, char *argv[], CommandLineArguments& args)
{
  // Specify the possible options.
  po::options_description genericOptions("Generic options");
  genericOptions.add_options()
    ("help", "produce help message")
    ("batch", po::bool_switch(&args.batch), "don't wait for user input before starting the reconstruction and terminate immediately")
    ("calib,c", po::value<std::string>(&args.calibrationFilename)->default_value(""), "calibration filename")
    ("cameraAfterDisk", po::bool_switch(&args.cameraAfterDisk), "switch to the camera after a disk sequence")
    ("configFile,f", po::value<std::string>(), "additional parameters filename")
    ("detectFiducials", po::bool_switch(&args.detectFiducials), "enable fiducial detection")
    ("experimentTag", po::value<std::string>(&args.experimentTag)->default_value(""), "experiment tag")
    ("mapSurfels", po::bool_switch(&args.mapSurfels), "enable surfel mapping")
    ("noRelocaliser", po::bool_switch(&args.noRelocaliser), "don't use the relocaliser")
    ("noTracker", po::bool_switch(&args.noTracker), "don't use any tracker")
    ("pipelineType", po::value<std::string>(&args.pipelineType)->default_value("semantic"), "pipeline type")
    ("renderFiducials", po::bool_switch(&args.renderFiducials), "enable fiducial rendering")
    ("saveMeshOnExit", po::bool_switch(&args.saveMeshOnExit), "save reconstructed mesh on exit")
    ("trackSurfels", po::bool_switch(&args.trackSurfels), "enable surfel mapping and tracking")
  ;

  po::options_description cameraOptions("Camera options");
  cameraOptions.add_options()
    ("uri,u", po::value<std::string>(&args.openNIDeviceURI)->default_value("Default"), "OpenNI device URI")
  ;

  po::options_description diskSequenceOptions("Disk sequence options");
  diskSequenceOptions.add_options()
    ("depthMask,d", po::value<std::vector<std::string> >(&args.depthImageMask)->multitoken(), "depth image mask")
    ("initialFrame,n", po::value<int>(&args.initialFrameNumber)->default_value(0), "initial frame number")
    ("poseFromDisk", po::bool_switch(&args.poseFromDisk), "track the camera using poses stored on disk")
    ("poseMask,p", po::value<std::string>(&args.poseFilesMask)->default_value(""), "pose files mask")
    ("prefetchBufferCapacity,b", po::value<size_t>(&args.prefetchBufferCapacity)->default_value(60), "capacity of the prefetch buffer")
    ("rgbMask,r", po::value<std::vector<std::string> >(&args.rgbImageMask)->multitoken(), "RGB image mask")
    ("sequenceSpecifier,s", po::value<std::vector<std::string> >(&args.sequenceSpecifier)->multitoken(), "sequence specifier")
    ("sequenceType", po::value<std::string>(&args.sequenceType)->default_value("sequence"), "sequence type")
  ;

  po::options_description options;
  options.add(genericOptions);
  options.add(cameraOptions);
  options.add(diskSequenceOptions);

  // Actually parse the command line.
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, options), vm);

  // Parse options from configuration file, if necessary.
  if(vm.count("configFile"))
  {
    // Allow unregistered options: those are added to the global ParametersContainer instance, to be used by other classes.
    po::parsed_options parsedOptions = po::parse_config_file<char>(vm["configFile"].as<std::string>().c_str(), options, true);

    // Store registered options in the variable map
    po::store(parsedOptions, vm);

    // Check for unregistered options
    ParametersContainer &parametersContainer = ParametersContainer::instance();
    for(size_t optionIdx = 0; optionIdx < parsedOptions.options.size(); ++optionIdx)
    {
      const po::basic_option<char> &option = parsedOptions.options[optionIdx];
      if (option.unregistered)
      {
        // Add all values in order
        for(size_t valueIdx = 0; valueIdx < option.value.size(); ++valueIdx)
        {
          parametersContainer.add_value(option.string_key, option.value[valueIdx]);
        }
      }
    }
  }

  po::notify(vm);

  // If the user specifies the --help flag, print a help message.
  if(vm.count("help"))
  {
    std::cout << options << '\n';
    return false;
  }

  // If the user specifies a sequence (either via a sequence name or a path),
  // set the depth / RGB image masks and the calibration filename appropriately.
  if(!args.sequenceSpecifier.empty())
  {
    args.depthImageMask.clear();
    args.rgbImageMask.clear();

    for(size_t i = 0; i < args.sequenceSpecifier.size(); ++i)
    {
      const std::string currentSequenceSpecifier = args.sequenceSpecifier[i];
      const bf::path dir = bf::is_directory(currentSequenceSpecifier)
        ? currentSequenceSpecifier
        : find_subdir_from_executable(args.sequenceType + "s") / currentSequenceSpecifier;

      args.depthImageMask.push_back((dir / "depthm%06i.pgm").string());
      args.rgbImageMask.push_back((dir / "rgbm%06i.ppm").string());
    }

    const std::string firstSequenceSpecifier = args.sequenceSpecifier[0];
    const bf::path firstSequenceDir = bf::is_directory(firstSequenceSpecifier)
      ? firstSequenceSpecifier
      : find_subdir_from_executable(args.sequenceType + "s") / firstSequenceSpecifier;

    // If the user hasn't explicitly specified a calibration file,
    // try to find one in the FIRST sequence directory.
    if(args.calibrationFilename == "")
    {
      bf::path defaultCalibrationFilename = firstSequenceDir / "calib.txt";
      if(bf::exists(defaultCalibrationFilename))
      {
        args.calibrationFilename = defaultCalibrationFilename.string();
      }
    }

    // Poses from disk are supported only for the first sequence at the moment
    if (args.poseFromDisk)
    {
      args.poseFilesMask = (firstSequenceDir / "posem%06i.txt").string();
    }
  }

  // If the user wants to enable surfel tracking, make sure that surfel mapping is also enabled.
  if(args.trackSurfels) args.mapSurfels = true;

  // If the user wants to enable fiducial rendering, make sure that fiducial detection is also enabled.
  if(args.renderFiducials) args.detectFiducials = true;

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
  settings->trackerConfig = NULL;

  // 7scenes GT
  settings->sceneParams.mu = 0.04f;
  settings->sceneParams.voxelSize = 0.01f;
  settings->sceneParams.viewFrustum_max = 5.0f;

  if(args.cameraAfterDisk || !args.noRelocaliser) settings->behaviourOnFailure = ITMLibSettings::FAILUREMODE_RELOCALISE;

  TrackerType trackerType = TRACKER_INFINITAM;
  std::vector<std::string> trackerConfigs;

  if (!args.poseFilesMask.empty())
  {
    std::cout << "[spaint] Reading poses from disk: " << args.poseFilesMask << '\n';
    trackerType = TRACKER_INFINITAM_NO_REFINE;
    trackerConfigs.push_back("type=file,mask=" + args.poseFilesMask);
  }

  // If we're trying to use the Rift tracker:
  if(trackerType == TRACKER_RIFT)
  {
#ifdef WITH_OVR
    // If the Rift isn't available when the program runs, make sure that we're not trying to use the Rift tracker.
    if(ovrHmd_Detect() == 0)
    {
      trackerType = TRACKER_INFINITAM;
    }
    else
    {
      trackerConfigs.push_back("TRACKER_RIFT dummy config entry");
    }
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
    trackerParams.push_back("192.168.10.1:801");

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

  if(args.noTracker)
  {
    std::cout << "[spaint] Online camera tracking disabled.\n";
  }
  else
  {
    // Setup default tracker (last one in the configuration vector).
    if(args.trackSurfels) trackerConfigs.push_back("type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=0,framesToWeight=1,failureDec=20.0");
    else trackerConfigs.push_back("type=extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0");
  }

  // Pass the device type to the memory block factory.
  MemoryBlockFactory::instance().set_device_type(settings->deviceType);

  // Construct the image source engine.
  boost::shared_ptr<CompositeImageSourceEngine> imageSourceEngine(new CompositeImageSourceEngine);

  // Instantiate an engine for each pair of image masks provided
  for(size_t i = 0; i < args.depthImageMask.size(); ++i)
  {
    const std::string depthImageMask = args.depthImageMask[i];
    const std::string rgbImageMask = args.rgbImageMask[i];

    std::cout << "[spaint] Reading images from disk: " << rgbImageMask << ' ' << depthImageMask << '\n';
    ImageMaskPathGenerator pathGenerator(rgbImageMask.c_str(), depthImageMask.c_str());
    imageSourceEngine->addSubengine(new AsyncImageSourceEngine(
      new ImageFileReader<ImageMaskPathGenerator>(args.calibrationFilename.c_str(), pathGenerator, args.initialFrameNumber),
      args.prefetchBufferCapacity
    ));
  }

  if(args.depthImageMask.empty() || args.cameraAfterDisk)
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

  // Construct the fiducial detector (if any).
  FiducialDetector_CPtr fiducialDetector;
#ifdef WITH_OPENCV
  fiducialDetector.reset(new ArUcoFiducialDetector(settings));
#endif

  // Construct the pipeline.
  const size_t maxLabelCount = 10;
  SLAMComponent::MappingMode mappingMode = args.mapSurfels ? SLAMComponent::MAP_BOTH : SLAMComponent::MAP_VOXELS_ONLY;
  SLAMComponent::TrackingMode trackingMode = args.trackSurfels ? SLAMComponent::TRACK_SURFELS : SLAMComponent::TRACK_VOXELS;

  MultiScenePipeline_Ptr pipeline;
  if(args.pipelineType == "semantic")
  {
    const unsigned int seed = 12345;
    pipeline.reset(new SemanticPipeline(
      settings,
      Application::resources_dir().string(),
      maxLabelCount,
      imageSourceEngine,
      seed,
      trackerType,
      trackerConfigs,
      mappingMode,
      trackingMode,
      fiducialDetector,
      args.detectFiducials
    ));
  }
  else if(args.pipelineType == "slam")
  {
    pipeline.reset(new SLAMPipeline(settings,
                                    args.experimentTag,
                                    Application::resources_dir().string(),
                                    imageSourceEngine,
                                    trackerType,
                                    trackerConfigs,
                                    mappingMode,
                                    trackingMode));
  }
  else throw std::runtime_error("Unknown pipeline type: " + args.pipelineType);

  // Configure the application.
  Application app(pipeline, args.renderFiducials);
  app.set_save_mesh_on_exit(args.saveMeshOnExit);
  app.set_batch_mode(args.batch);

  // Run the application.
  bool runSucceeded = app.run();

#ifdef WITH_OVR
  // If we built with Rift support, shut down the Rift SDK.
  ovr_Shutdown();
#endif

  // Shut down SDL.
  SDL_Quit();

  return runSucceeded ? EXIT_SUCCESS : EXIT_FAILURE;
}
catch(std::exception& e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
