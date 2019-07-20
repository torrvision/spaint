/**
 * spaintgui: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

// Note: This must appear before anything that could include SDL.h, since it includes boost/asio.hpp, a header that has a WinSock conflict with SDL.h.
#include "Application.h"

#if defined(WITH_ARRAYFIRE) && defined(WITH_CUDA)
  #ifdef _MSC_VER
    // Suppress a VC++ warning that is produced when including ArrayFire headers.
    #pragma warning(disable:4275)
  #endif

  #include <af/cuda.h>

  #ifdef _MSC_VER
    // Reenable the suppressed warning for the rest of the translation unit.
    #pragma warning(default:4275)
  #endif
#endif

#include <InputSource/IdleImageSourceEngine.h>
#include <InputSource/OpenNIEngine.h>
#ifdef WITH_LIBROYALE
#include <InputSource/PicoFlexxEngine.h>
#endif
#ifdef WITH_REALSENSE
#include <InputSource/RealSenseEngine.h>
#endif

#ifdef WITH_GLUT
#include <oglx/WrappedGLUT.h>
#endif

#ifdef WITH_OVR
#include <OVR_CAPI.h>
#endif

#include <itmx/imagesources/AsyncImageSourceEngine.h>
#include <itmx/imagesources/RemoteImageSourceEngine.h>
#ifdef WITH_ZED
#include <itmx/imagesources/ZedImageSourceEngine.h>
#endif

#include <orx/base/MemoryBlockFactory.h>
#include <orx/geometry/GeometryUtil.h>

#include <tvgutil/filesystem/PathFinder.h>

#include "core/CollaborativePipeline.h"
#include "core/ObjectivePipeline.h"
#include "core/SemanticPipeline.h"
#include "core/SLAMPipeline.h"
#include "sequences/SpaintSequence.h"

using namespace InputSource;
using namespace ITMLib;

using namespace itmx;
using namespace orx;
using namespace spaint;
using namespace tvgutil;

//#################### NAMESPACE ALIASES ####################

namespace bf = boost::filesystem;
namespace po = boost::program_options;

//#################### TYPES ####################

struct CommandLineArguments
{
  //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~

  // User-specifiable arguments
  bool batch;
  std::string calibrationFilename;
  bool cameraAfterDisk;
  std::string collaborationMode;
  std::vector<std::string> depthImageMasks;
  std::vector<float> depthNoiseSigmas;
  bool detectFiducials;
  std::string experimentTag;
  std::string fiducialDetectorType;
  std::string globalPosesSpecifier;
  bool headless;
  std::string host;
  std::vector<size_t> initialFrameNumbers;
  std::string leapFiducialID;
  bool mapSurfels;
  std::vector<double> missingDepthFractions;
  std::string modelSpecifier;
  bool noRelocaliser;
  std::string openNIDeviceURI;
  std::string pipelineType;
  std::string port;
  std::vector<std::string> poseFileMasks;
  size_t prefetchBufferCapacity;
  bool profileMemory;
  std::string relocaliserType;
  bool renderFiducials;
  std::vector<std::string> rgbImageMasks;
  bool runServer;
  bool saveMeshOnExit;
  bool saveModelsOnExit;
  std::vector<std::string> semanticImageMasks;
  std::vector<std::string> sequenceSpecifiers;
  std::vector<std::string> sequenceTypes;
  std::string subwindowConfigurationIndex;
  std::vector<std::string> trackerSpecifiers;
  bool trackObject;
  bool trackSurfels;
  bool useVicon;
  bool verbose;
  std::string viconHost;

  // Derived arguments
  boost::optional<bf::path> modelDir;
  std::vector<Sequence_CPtr> sequences;

  //~~~~~~~~~~~~~~~~~~~~ PUBLIC MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~

  /**
   * \brief Adds the command-line arguments to a settings object.
   *
   * \param settings  The settings object.
   */
  void add_to_settings(const Settings_Ptr& settings)
  {
    std::vector<std::string> diskTrackerConfigs;
    for(size_t i = 0, size = sequences.size(); i < size; ++i)
    {
      diskTrackerConfigs.push_back(sequences[i]->make_disk_tracker_config());
    }

    #define ADD_SETTING(arg) settings->add_value(#arg, boost::lexical_cast<std::string>(arg))
    #define ADD_SETTINGS(arg) for(size_t i = 0; i < arg.size(); ++i) { settings->add_value(#arg, boost::lexical_cast<std::string>(arg[i])); }
      ADD_SETTING(batch);
      ADD_SETTING(calibrationFilename);
      ADD_SETTING(collaborationMode);
      ADD_SETTINGS(depthImageMasks);
      ADD_SETTINGS(depthNoiseSigmas);
      ADD_SETTING(detectFiducials);
      ADD_SETTINGS(diskTrackerConfigs);
      ADD_SETTING(experimentTag);
      ADD_SETTING(fiducialDetectorType);
      ADD_SETTING(globalPosesSpecifier);
      ADD_SETTING(headless);
      ADD_SETTING(host);
      ADD_SETTINGS(initialFrameNumbers);
      ADD_SETTING(leapFiducialID);
      ADD_SETTING(mapSurfels);
      ADD_SETTINGS(missingDepthFractions);
      ADD_SETTING(modelSpecifier);
      ADD_SETTING(noRelocaliser);
      ADD_SETTING(openNIDeviceURI);
      ADD_SETTING(pipelineType);
      ADD_SETTING(port);
      ADD_SETTINGS(poseFileMasks);
      ADD_SETTING(prefetchBufferCapacity);
      ADD_SETTING(profileMemory);
      ADD_SETTING(relocaliserType);
      ADD_SETTING(renderFiducials);
      ADD_SETTINGS(rgbImageMasks);
      ADD_SETTING(runServer);
      ADD_SETTING(saveMeshOnExit);
      ADD_SETTING(saveModelsOnExit);
      ADD_SETTINGS(semanticImageMasks);
      ADD_SETTINGS(sequenceSpecifiers);
      ADD_SETTINGS(sequenceTypes);
      ADD_SETTING(subwindowConfigurationIndex);
      ADD_SETTINGS(trackerSpecifiers);
      ADD_SETTING(trackObject);
      ADD_SETTING(trackSurfels);
      ADD_SETTING(useVicon);
      ADD_SETTING(verbose);
      ADD_SETTING(viconHost);
    #undef ADD_SETTINGS
    #undef ADD_SETTING
  }
};

//#################### FUNCTIONS ####################

/**
 * \brief Adds any unregistered options in a set of parsed options to a settings object.
 *
 * \param parsedOptions The set of parsed options.
 * \param settings      The settings object.
 */
void add_unregistered_options_to_settings(const po::parsed_options& parsedOptions, const Settings_Ptr& settings)
{
  for(size_t i = 0, optionCount = parsedOptions.options.size(); i < optionCount; ++i)
  {
    const po::basic_option<char>& option = parsedOptions.options[i];
    if(option.unregistered)
    {
      // Add all the specified values for the option in the correct order.
      for(size_t j = 0, valueCount = option.value.size(); j < valueCount; ++j)
      {
        settings->add_value(option.string_key, option.value[j]);
      }
    }
  }
}

/**
 * \brief Checks whether or not the specified camera subengine is able to provide depth images.
 *
 * \note If the check fails, the camera subengine will be deallocated.
 *
 * \param cameraSubengine The camera subengine to check.
 * \return                The camera subengine, if it is able to provide depth images, or NULL otherwise.
 */
ImageSourceEngine *check_camera_subengine(ImageSourceEngine *cameraSubengine)
{
  if(cameraSubengine->getDepthImageSize().x == 0)
  {
    delete cameraSubengine;
    return NULL;
  }
  else return cameraSubengine;
}

/**
 * \brief Copies any (voxel) scene parameters that have been specified in the configuration file across to the actual scene parameters object.
 *
 * \param settings  The settings for the application.
 */
void copy_scene_params(const Settings_Ptr& settings)
{
#define COPY_PARAM(type, name, defaultValue) settings->sceneParams.name = settings->get_first_value<type>("SceneParams."#name, defaultValue)

  // Note: The default values are taken from InfiniTAM.
  COPY_PARAM(int, maxW, 100);
  COPY_PARAM(float, mu, 0.02f);
  COPY_PARAM(bool, stopIntegratingAtMaxW, false);
  COPY_PARAM(float, viewFrustum_max, 3.0f);
  COPY_PARAM(float, viewFrustum_min, 0.2f);
  COPY_PARAM(float, voxelSize, 0.005f);

#undef COPY_PARAM
}

/**
 * \brief Copies any surfel scene parameters that have been specified in the configuration file across to the actual surfel scene parameters object.
 *
 * \param settings  The settings for the application.
 */
void copy_surfel_scene_params(const Settings_Ptr& settings)
{
#define COPY_PARAM(type, name, defaultValue) settings->surfelSceneParams.name = settings->get_first_value<type>("SurfelSceneParams."#name, defaultValue)

  // Note: The default values are taken from InfiniTAM.
  COPY_PARAM(float, deltaRadius, 0.5f);
  COPY_PARAM(float, gaussianConfidenceSigma, 0.6f);
  COPY_PARAM(float, maxMergeAngle, static_cast<float>(20 * M_PI / 180));
  COPY_PARAM(float, maxMergeDist, 0.01f);
  COPY_PARAM(float, maxSurfelRadius, 0.004f);
  COPY_PARAM(float, minRadiusOverlapFactor, 3.5f);
  COPY_PARAM(float, stableSurfelConfidence, 25.0f);
  COPY_PARAM(int, supersamplingFactor, 4);
  COPY_PARAM(float, trackingSurfelMaxDepth, 1.0f);
  COPY_PARAM(float, trackingSurfelMinConfidence, 5.0f);
  COPY_PARAM(int, unstableSurfelPeriod, 20);
  COPY_PARAM(int, unstableSurfelZOffset, 10000000);
  COPY_PARAM(bool, useGaussianSampleConfidence, true);
  COPY_PARAM(bool, useSurfelMerging, true);

#undef COPY_PARAM
}

/**
 * \brief Determines the sequences (if any) that the user wants to load from disk, based on the program's command-line arguments.
 *
 * \param args  The program's command-line arguments.
 * \return      The sequences (if any) that the user wants to load from disk.
 */
std::vector<Sequence_CPtr> determine_sequences(const CommandLineArguments& args)
{
  std::vector<Sequence_CPtr> sequences;

  // If the numbers of depth image and RGB image masks don't match, throw an error.
  if(args.depthImageMasks.size() != args.rgbImageMasks.size())
  {
    throw std::runtime_error("Error: The numbers of depth image and RGB image masks must match");
  }

  // Add any sequence that the user specifies using a sequence specifier.
  for(size_t i = 0, size = args.sequenceSpecifiers.size(); i < size; ++i)
  {
    size_t initialFrameNumber = i < args.initialFrameNumbers.size() ? args.initialFrameNumbers[i] : 0;
    double missingDepthFraction = i < args.missingDepthFractions.size() ? args.missingDepthFractions[i] : 0.0;
    float depthNoiseSigma = i < args.depthNoiseSigmas.size() ? args.depthNoiseSigmas[i] : 0.0f;

    const std::string& sequenceSpecifier = args.sequenceSpecifiers[i];
    if(!bf::is_regular_file(sequenceSpecifier))
    {
      // Determine the sequence type.
      const std::string sequenceType = i < args.sequenceTypes.size() ? args.sequenceTypes[i] : "sequence";

      // Determine the directory containing the sequence.
      bf::path dir = bf::is_directory(sequenceSpecifier)
        ? sequenceSpecifier
        : find_subdir_from_executable(sequenceType + "s") / sequenceSpecifier;

      // Add the sequence to the list.
      sequences.push_back(Sequence_CPtr(new SpaintSequence(dir, initialFrameNumber, missingDepthFraction, depthNoiseSigma)));
    }
    else throw std::runtime_error("Error: The sequence specifier '" + sequenceSpecifier + "' denotes a file rather than a directory");
  }

  // Add any sequence that the user specifies implicitly via depth/RGB/pose masks.
  for(size_t i = 0, size = args.depthImageMasks.size(); i < size; ++i)
  {
    std::string depthImageMask = args.depthImageMasks[i];
    std::string rgbImageMask = args.rgbImageMasks[i];
    std::string poseFileMask = i < args.poseFileMasks.size() ? args.poseFileMasks[i] : "";
    std::string semanticImageMask = i < args.semanticImageMasks.size() ? args.semanticImageMasks[i] : "";

    const size_t j = i + args.sequenceSpecifiers.size();
    size_t initialFrameNumber = j < args.initialFrameNumbers.size() ? args.initialFrameNumbers[j] : 0;
    double missingDepthFraction = j < args.missingDepthFractions.size() ? args.missingDepthFractions[j] : 0.0;
    float depthNoiseSigma = j < args.depthNoiseSigmas.size() ? args.depthNoiseSigmas[j] : 0.0f;

    sequences.push_back(Sequence_CPtr(new SpaintSequence(depthImageMask, rgbImageMask, poseFileMask, semanticImageMask, initialFrameNumber, missingDepthFraction, depthNoiseSigma)));
  }

  return sequences;
}

/**
 * \brief Attempts to load a set of global poses from a file specified by a global poses specifier.
 *
 * \param globalPosesSpecifier  The global poses specifier.
 * \return                      The global poses from the file, if possible, or an empty map otherwise.
 */
std::map<std::string,DualQuatd> load_global_poses(const std::string& globalPosesSpecifier)
{
  std::map<std::string,DualQuatd> globalPoses;

  // Determine the file from which to load the global poses.
  const std::string dirName = "global_poses";
  const bf::path p = bf::is_regular(globalPosesSpecifier) ? globalPosesSpecifier : find_subdir_from_executable(dirName) / (globalPosesSpecifier + ".txt");

  // Try to read the poses from the file. If we can't, throw.
  std::ifstream fs(p.string().c_str());
  if(!fs) throw std::runtime_error("Error: Could not open global poses file");

  std::string id;
  DualQuatd dq;
  while(fs >> id >> dq)
  {
    globalPoses.insert(std::make_pair(id, dq));
  }

  return globalPoses;
}

/**
 * \brief Attempts to make a camera subengine to read images from any suitable camera that is attached.
 *
 * \param args  The program's command-line arguments.
 * \return      The camera subengine, if a suitable camera is attached, or NULL otherwise.
 */
ImageSourceEngine *make_camera_subengine(const CommandLineArguments& args)
{
  ImageSourceEngine *cameraSubengine = NULL;

#ifdef WITH_OPENNI
  // Probe for an OpenNI camera.
  if(cameraSubengine == NULL)
  {
    std::cout << "[spaint] Probing OpenNI camera: " << args.openNIDeviceURI << '\n';
    boost::optional<std::string> uri = args.openNIDeviceURI == "Default" ? boost::none : boost::optional<std::string>(args.openNIDeviceURI);
    bool useInternalCalibration = !uri; // if reading from a file, assume that the provided calibration is to be used
    cameraSubengine = check_camera_subengine(new OpenNIEngine(args.calibrationFilename.c_str(), uri ? uri->c_str() : NULL, useInternalCalibration
#if USE_LOW_USB_BANDWIDTH_MODE
      // If there is insufficient USB bandwidth available to support 640x480 RGB input, use 320x240 instead.
      , Vector2i(320, 240)
#endif
    ));
  }
#endif

#if WITH_LIBROYALE
  // Probe for a PicoFlexx camera.
  if(cameraSubengine == NULL)
  {
    std::cout << "[spaint] Probing PicoFlexx camera\n";
    cameraSubengine = check_camera_subengine(new PicoFlexxEngine(""));
  }
#endif

#ifdef WITH_REALSENSE
  // Probe for a RealSense camera.
  if(cameraSubengine == NULL)
  {
    std::cout << "[spaint] Probing RealSense camera\n";
    cameraSubengine = check_camera_subengine(new RealSenseEngine(args.calibrationFilename.c_str()));
  }
#endif

#ifdef WITH_ZED
  // Probe for a Zed camera.
  if(cameraSubengine == NULL)
  {
    std::cout << "[spaint] Probing Zed camera\n";
    cameraSubengine = check_camera_subengine(new ZedImageSourceEngine(ZedCamera::instance()));
  }
#endif

  return cameraSubengine;
}

boost::shared_ptr<CompositeImageSourceEngine> make_image_source_engine(const CommandLineArguments& args)
{
  boost::shared_ptr<CompositeImageSourceEngine> imageSourceEngine(new CompositeImageSourceEngine);

  // If a model was specified without either a disk sequence or the camera following it, add an idle subengine to allow the model to still be viewed.
  if(args.modelDir && args.sequences.empty() && !args.cameraAfterDisk)
  {
    const std::string calibrationFilename = (*args.modelDir / "calib.txt").string();
    imageSourceEngine->addSubengine(new IdleImageSourceEngine(calibrationFilename.c_str()));
  }

  // Add a subengine for each disk sequence specified.
  for(size_t i = 0; i < args.sequences.size(); ++i)
  {
    // If no calibration file was specified by the user, and the first disk sequence has a calibration file, use that.
    // FIXME: It would be better to use the correct calibration file for each disk sequence, but our pipeline doesn't
    //        yet support changing the camera calibration parameters during reconstruction.
    const bf::path calibrationPath = args.sequences[0]->default_calib_path();
    const std::string calibrationFilename = (args.calibrationFilename != "" || !bf::exists(calibrationPath)) ? args.calibrationFilename : calibrationPath.string();

    std::cout << "[spaint] Reading images from disk: " << *args.sequences[i] << '\n';
    imageSourceEngine->addSubengine(new AsyncImageSourceEngine(args.sequences[i]->make_image_source_engine(calibrationFilename), args.prefetchBufferCapacity));
  }

  // If no model and no disk sequences were specified, or we want to switch to the camera once all the disk sequences finish, add a camera subengine.
  if((!args.modelDir && args.sequences.empty()) || args.cameraAfterDisk)
  {
    ImageSourceEngine *cameraSubengine = make_camera_subengine(args);
    if(cameraSubengine != NULL) imageSourceEngine->addSubengine(cameraSubengine);
  }

  return imageSourceEngine;
}

/**
 * \brief Makes the overall tracker configuration based on any tracker specifiers that were passed in on the command line.
 *
 * \param args  The program's command-line arguments.
 * \return      The overall tracker configuration.
 */
std::string make_tracker_config(const CommandLineArguments& args)
{
  std::string result;

  // If the user wants to use global poses for the scenes, load them from disk.
  std::map<std::string,DualQuatd> globalPoses;
  if(args.globalPosesSpecifier != "") globalPoses = load_global_poses(args.globalPosesSpecifier);

  // Determine the number of different trackers that will be needed.
  size_t trackerCount = args.sequences.size();
  if(trackerCount == 0 || args.cameraAfterDisk) ++trackerCount;

  // If more than one tracker is needed, make the overall tracker a composite.
  if(trackerCount > 1) result += "<tracker type='composite' policy='sequential'>";

  // For each tracker that is needed:
  for(size_t i = 0; i < trackerCount; ++i)
  {
    // Look to see if the user specified an explicit tracker specifier for it on the command line; if not, use a default tracker specifier.
    const std::string trackerSpecifier = i < args.trackerSpecifiers.size() ? args.trackerSpecifiers[i] : "InfiniTAM";

    // Separate the tracker specifier into chunks.
    typedef boost::char_separator<char> sep;
    typedef boost::tokenizer<sep> tokenizer;

    tokenizer tok(trackerSpecifier.begin(), trackerSpecifier.end(), sep("+"));
    std::vector<std::string> chunks(tok.begin(), tok.end());

    // Add a tracker configuration based on the specifier chunks to the overall tracker configuration.
    // If more than one chunk is involved, bundle the subsidiary trackers into a refining composite.
    size_t chunkCount = chunks.size();
    if(chunkCount > 1) result += "<tracker type='composite'>";

    for(size_t j = 0; j < chunkCount; ++j)
    {
      if(chunks[j] == "InfiniTAM")
      {
        result += "<tracker type='infinitam'/>";
      }
      else if(chunks[j] == "Disk")
      {
        // If we're using global poses for the scenes:
        if(!globalPoses.empty())
        {
          // Try to find the global pose for this scene based on the sequence ID.
          const std::string sequenceID = args.sequences[i]->id();
          std::map<std::string,DualQuatd>::const_iterator it = globalPoses.find(sequenceID);

          // If that doesn't work, try to find the global pose based on the scene ID.
          if(it == globalPoses.end())
          {
            // FIXME: We shouldn't hard-code "Local" here - it's based on knowing how CollaborativePipeline assigns scene names.
            const std::string sceneID = i == 0 ? Model::get_world_scene_id() : "Local" + boost::lexical_cast<std::string>(i);
            it = globalPoses.find(sceneID);
          }

          // If we now have a global pose, specify the creation of a global tracker that uses it. If not, throw.
          if(it != globalPoses.end()) result += "<tracker type='global'><params>" + boost::lexical_cast<std::string>(it->second) + "</params>";
          else throw std::runtime_error("Error: Global pose for sequence '" + sequenceID + "' not found");
        }

        // Specify the creation of a disk-based tracker that reads poses from disk.
        result += args.sequences[i]->make_disk_tracker_config();

        // If we're using global poses for the scenes, add the necessary closing tag for the global tracker.
        if(!globalPoses.empty()) result += "</tracker>";
      }
      else
      {
        result += "<tracker type='import'><params>builtin:" + chunks[j] + "</params></tracker>";
      }
    }

    // If more than one chunk was involved, add the necessary closing tag for the refining composite.
    if(chunkCount > 1) result += "</tracker>";
  }

  // If more than one tracker was needed, add the necessary closing tag for the overall composite.
  if(trackerCount > 1) result += "</tracker>";

  return result;
}

/**
 * \brief Parses a configuration file and adds its registered options to the application's variables map
 *        and its unregistered options to the application's settings.
 *
 * \param filename  The name of the configuration file.
 * \param options   The registered options for the application.
 * \param vm        The variables map for the application.
 * \param settings  The settings for the application.
 */
void parse_configuration_file(const std::string& filename, const po::options_description& options, po::variables_map& vm, const Settings_Ptr& settings)
{
  // Parse the options in the configuration file.
  po::parsed_options parsedConfigFileOptions = po::parse_config_file<char>(filename.c_str(), options, true);

  // Add any registered options to the variables map.
  po::store(parsedConfigFileOptions, vm);

  // Add any unregistered options to the settings.
  add_unregistered_options_to_settings(parsedConfigFileOptions, settings);
}

/**
 * \brief Post-process the program's command-line arguments and add them to the application settings.
 *
 * \param args      The program's command-line arguments.
 * \param options   The registered options for the application.
 * \param vm        The variables map for the application.
 * \param settings  The settings for the application.
 */
void postprocess_arguments(CommandLineArguments& args, const po::options_description& options, po::variables_map& vm, const Settings_Ptr& settings)
{
  // Determine the sequences (if any) that the user wants to load from disk, based on the program's command-line arguments.
  args.sequences = determine_sequences(args);

  // If the user specified a model to load, determine the model directory and parse the model's configuration file (if present).
  if(args.modelSpecifier != "")
  {
    args.modelDir = bf::is_directory(args.modelSpecifier) ? args.modelSpecifier : find_subdir_from_executable("models") / args.modelSpecifier / Model::get_world_scene_id();

    const bf::path configPath = *args.modelDir / "settings.ini";
    if(bf::is_regular_file(configPath))
    {
      // Parse any additional options from the model's configuration file.
      parse_configuration_file(configPath.string(), options, vm, settings);
      po::notify(vm);
    }
  }

  // If the user wants to use global poses for the scenes, make sure that each disk sequence has a tracker specifier set to Disk.
  if(args.globalPosesSpecifier != "")
  {
    args.trackerSpecifiers.resize(args.sequenceSpecifiers.size());
    for(size_t i = 0, size = args.sequenceSpecifiers.size(); i < size; ++i)
    {
      args.trackerSpecifiers[i] = "Disk";
    }
  }

  // If the user wants to enable surfel tracking, make sure that surfel mapping is also enabled.
  if(args.trackSurfels) args.mapSurfels = true;

  // If the user wants to enable fiducial rendering or specifies a fiducial to use for the Leap Motion,
  // make sure that fiducial detection is enabled.
  if(args.renderFiducials || args.leapFiducialID != "")
  {
    args.detectFiducials = true;
  }

  // If the user wants to run in headless mode, make sure that batch mode is also enabled
  // (there is no way to control the application without the UI anyway).
  if(args.headless) args.batch = true;

  // If the user wants to use a Vicon fiducial detector or a Vicon-based tracker, make sure that the Vicon system it needs is enabled.
  if(args.fiducialDetectorType == "vicon")
  {
    args.useVicon = true;
  }

  for(size_t i = 0, size = args.trackerSpecifiers.size(); i < size; ++i)
  {
    const std::string trackerSpecifier = boost::to_lower_copy(args.trackerSpecifiers[i]);
    if(trackerSpecifier.find("vicon") != std::string::npos)
    {
      args.useVicon = true;
    }
  }

  // If the user wants to use a collaborative pipeline, but doesn't specify any disk sequences,
  // make sure a mapping server is started.
  if(args.pipelineType == "collaborative" && args.sequenceSpecifiers.empty())
  {
    args.runServer = true;
  }

  // If the user tries to run the application in both batch mode and server mode simultaneously, throw.
  // It doesn't make sense to combine the two modes: server mode is intended to make sure that fusion
  // starts as soon as frames arrive from a client; batch mode is intended to make sure that the user
  // cannot quit the application during experiments, and that the application quits automatically once
  // an experiment is finished. Both modes initially unpause the fusion process, but they are otherwise
  // intended for completely different use cases and should not be combined (indeed, they conflict).
  if(args.batch && args.runServer)
  {
    throw std::runtime_error("Error: Cannot enable both batch mode and server mode at the same time.");
  }

  // Add the post-processed arguments to the application settings.
  args.add_to_settings(settings);
}

/**
 * \brief Parses any command-line arguments passed in by the user and adds them to the application settings.
 *
 * \param argc      The command-line argument count.
 * \param argv      The raw command-line arguments.
 * \param args      The parsed command-line arguments.
 * \param settings  The application settings.
 * \return          true, if the program should continue after parsing the command-line arguments, or false otherwise.
 */
bool parse_command_line(int argc, char *argv[], CommandLineArguments& args, const Settings_Ptr& settings)
{
  // Specify the possible options.
  po::options_description genericOptions("Generic options");
  genericOptions.add_options()
    ("help", "produce help message")
    ("batch", po::bool_switch(&args.batch), "enable batch mode")
    ("calib,c", po::value<std::string>(&args.calibrationFilename)->default_value(""), "calibration filename")
    ("cameraAfterDisk", po::bool_switch(&args.cameraAfterDisk), "switch to the camera after a disk sequence")
    ("collaborationMode", po::value<std::string>(&args.collaborationMode)->default_value("batch"), "collaboration mode (batch|live)")
    ("configFile,f", po::value<std::string>(), "additional parameters filename")
    ("depthNoiseSigma", po::value<std::vector<float> >(&args.depthNoiseSigmas)->multitoken(), "depth noise sigma")
    ("detectFiducials", po::bool_switch(&args.detectFiducials), "enable fiducial detection")
    ("experimentTag", po::value<std::string>(&args.experimentTag)->default_value(Settings::NOT_SET), "experiment tag")
    ("fiducialDetectorType", po::value<std::string>(&args.fiducialDetectorType)->default_value("aruco"), "fiducial detector type (aruco|vicon)")
    ("globalPosesSpecifier,g", po::value<std::string>(&args.globalPosesSpecifier)->default_value(""), "global poses specifier")
    ("headless", po::bool_switch(&args.headless), "run in headless mode")
    ("host,h", po::value<std::string>(&args.host)->default_value(""), "remote mapping host")
    ("leapFiducialID", po::value<std::string>(&args.leapFiducialID)->default_value(""), "the ID of the fiducial to use for the Leap Motion")
    ("mapSurfels", po::bool_switch(&args.mapSurfels), "enable surfel mapping")
    ("missingDepthFraction", po::value<std::vector<double> >(&args.missingDepthFractions)->multitoken(), "missing depth fraction [0,1]")
    ("modelSpecifier,m", po::value<std::string>(&args.modelSpecifier)->default_value(""), "model specifier")
    ("noRelocaliser", po::bool_switch(&args.noRelocaliser), "don't use the relocaliser")
    ("pipelineType", po::value<std::string>(&args.pipelineType)->default_value("semantic"), "pipeline type")
    ("port", po::value<std::string>(&args.port)->default_value("7851"), "remote mapping port")
    ("profileMemory", po::bool_switch(&args.profileMemory)->default_value(false), "whether or not to profile the memory usage")
    ("relocaliserType", po::value<std::string>(&args.relocaliserType)->default_value("forest"), "relocaliser type")
    ("renderFiducials", po::bool_switch(&args.renderFiducials), "enable fiducial rendering")
    ("runServer", po::bool_switch(&args.runServer), "run a remote mapping server")
    ("saveMeshOnExit", po::bool_switch(&args.saveMeshOnExit), "save a mesh of the scene on exiting the application")
    ("saveModelsOnExit", po::bool_switch(&args.saveModelsOnExit), "save a model of each voxel scene on exiting the application")
    ("subwindowConfigurationIndex", po::value<std::string>(&args.subwindowConfigurationIndex)->default_value("1"), "subwindow configuration index")
    ("trackerSpecifier,t", po::value<std::vector<std::string> >(&args.trackerSpecifiers)->multitoken(), "tracker specifier")
    ("trackSurfels", po::bool_switch(&args.trackSurfels), "enable surfel mapping and tracking")
    ("useVicon", po::bool_switch(&args.useVicon)->default_value(false), "whether or not to use the Vicon system")
    ("verbose,v", po::bool_switch(&args.verbose), "enable verbose output")
    ("viconHost", po::value<std::string>(&args.viconHost)->default_value("192.168.0.101"), "Vicon host")
  ;

  po::options_description cameraOptions("Camera options");
  cameraOptions.add_options()
    ("uri,u", po::value<std::string>(&args.openNIDeviceURI)->default_value("Default"), "OpenNI device URI")
  ;

  po::options_description diskSequenceOptions("Disk sequence options");
  diskSequenceOptions.add_options()
    ("depthMask,d", po::value<std::vector<std::string> >(&args.depthImageMasks)->multitoken(), "depth image mask")
    ("initialFrame,n", po::value<std::vector<size_t> >(&args.initialFrameNumbers)->multitoken(), "initial frame numbers")
    ("poseMask,p", po::value<std::vector<std::string> >(&args.poseFileMasks)->multitoken(), "pose file mask")
    ("prefetchBufferCapacity,b", po::value<size_t>(&args.prefetchBufferCapacity)->default_value(60), "capacity of the prefetch buffer")
    ("rgbMask,r", po::value<std::vector<std::string> >(&args.rgbImageMasks)->multitoken(), "RGB image mask")
    ("sequenceSpecifier,s", po::value<std::vector<std::string> >(&args.sequenceSpecifiers)->multitoken(), "sequence specifier")
    ("sequenceType", po::value<std::vector<std::string> >(&args.sequenceTypes)->multitoken(), "sequence type")
  ;

  po::options_description objectivePipelineOptions("Objective pipeline options");
  objectivePipelineOptions.add_options()
    ("trackObject", po::bool_switch(&args.trackObject), "track the object")
  ;

  po::options_description options;
  options.add(genericOptions);
  options.add(cameraOptions);
  options.add(diskSequenceOptions);
  options.add(objectivePipelineOptions);

  // Parse the command line.
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, options), vm);

  // If a configuration file was specified:
  if(vm.count("configFile"))
  {
    // Parse additional options from the configuration file and add any registered options to the variables map.
    // These will be post-processed (if necessary) and added to the settings later. Unregistered options are
    // also allowed: we add these directly to the settings without post-processing.
    parse_configuration_file(vm["configFile"].as<std::string>(), options, vm, settings);
  }

  po::notify(vm);

  // Post-process any registered options and add them to the settings.
  postprocess_arguments(args, options, vm, settings);

  // Print the settings for the application so that the user can see them.
  std::cout << "Settings:\n" << *settings << '\n';

  // If the user specifies the --help flag, print a help message.
  if(vm.count("help"))
  {
    std::cout << options << '\n';
    return false;
  }

  return true;
}

/**
 * \brief Outputs the specified error message and terminates the program with the specified exit code.
 *
 * \param message The error message.
 * \param code    The exit code.
 */
void quit(const std::string& message, int code = EXIT_FAILURE)
{
  std::cerr << message << '\n';
  SDL_Quit();
  exit(code);
}

int main(int argc, char *argv[])
try
{
  // Construct the settings object for the application. This is used to store both the
  // settings for InfiniTAM and our own extended settings. Note that we do not use the
  // tracker configuration string in the InfiniTAM settings, and so we set it to NULL.
  Settings_Ptr settings(new Settings);
  settings->trackerConfig = NULL;

  // Parse the command-line arguments.
  CommandLineArguments args;
  if(!parse_command_line(argc, argv, args, settings))
  {
    return 0;
  }

  // If we're not running in headless mode, initialise the GUI-only subsystems.
  if(!args.headless)
  {
    // Initialise SDL.
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
    {
      quit("Error: Failed to initialise SDL.");
    }

  #ifdef WITH_GLUT
    // Initialise GLUT (used for text rendering only).
    glutInit(&argc, argv);
  #endif

  #ifdef WITH_OVR
    // If we built with Rift support, initialise the Rift SDK.
    ovr_Initialize();
  #endif
  }

  // Find all available joysticks and report the number found to the user.
  const int availableJoysticks = SDL_NumJoysticks();
  std::cout << "[spaint] Found " << availableJoysticks << " joysticks.\n";

  // Open all available joysticks.
  typedef boost::shared_ptr<SDL_Joystick> SDL_Joystick_Ptr;
  std::vector<SDL_Joystick_Ptr> joysticks;
  for(int i = 0; i < availableJoysticks; ++i)
  {
    SDL_Joystick *joystick = SDL_JoystickOpen(i);
    if(!joystick) throw std::runtime_error("Couldn't open joystick " + boost::lexical_cast<std::string>(i));

    std::cout << "[spaint] Opened joystick " << i << ": " << SDL_JoystickName(joystick) << '\n';
    joysticks.push_back(SDL_Joystick_Ptr(joystick, &SDL_JoystickClose));
  }

#if defined(WITH_ARRAYFIRE) && defined(WITH_CUDA)
  // Tell ArrayFire to run on the primary GPU.
  afcu::setNativeId(0);
#endif

  // Copy any scene parameters that have been set in the configuration file across to the actual scene parameters objects.
  copy_scene_params(settings);
  copy_surfel_scene_params(settings);

  // Set the failure behaviour of the relocaliser.
  if(args.cameraAfterDisk || !args.noRelocaliser) settings->behaviourOnFailure = ITMLibSettings::FAILUREMODE_RELOCALISE;

  // Pass the device type to the memory block factory.
  MemoryBlockFactory::instance().set_device_type(settings->deviceType);

  // Run a remote mapping server if requested.
  MappingServer_Ptr mappingServer;
  if(args.runServer)
  {
    const MappingServer::Mode mode = args.pipelineType == "collaborative" ? MappingServer::SM_MULTI_CLIENT : MappingServer::SM_SINGLE_CLIENT;
    mappingServer.reset(new MappingServer(mode));
    mappingServer->start();
  }

  // Construct the pipeline.
  MultiScenePipeline_Ptr pipeline;
  if(args.pipelineType != "collaborative")
  {
    const size_t maxLabelCount = 10;
    SLAMComponent::MappingMode mappingMode = args.mapSurfels ? SLAMComponent::MAP_BOTH : SLAMComponent::MAP_VOXELS_ONLY;
    SLAMComponent::TrackingMode trackingMode = args.trackSurfels ? SLAMComponent::TRACK_SURFELS : SLAMComponent::TRACK_VOXELS;

    if(args.pipelineType == "slam")
    {
      pipeline.reset(new SLAMPipeline(
        settings,
        Application::resources_dir().string(),
        make_image_source_engine(args),
        make_tracker_config(args),
        mappingMode,
        trackingMode,
        args.modelDir,
        args.detectFiducials
      ));
    }
    else if(args.pipelineType == "semantic")
    {
      const unsigned int seed = 12345;
      pipeline.reset(new SemanticPipeline(
        settings,
        Application::resources_dir().string(),
        maxLabelCount,
        make_image_source_engine(args),
        seed,
        make_tracker_config(args),
        mappingMode,
        trackingMode,
        args.modelDir,
        args.detectFiducials
      ));
    }
    else if(args.pipelineType == "objective")
    {
      pipeline.reset(new ObjectivePipeline(
        settings,
        Application::resources_dir().string(),
        maxLabelCount,
        make_image_source_engine(args),
        make_tracker_config(args),
        mappingMode,
        trackingMode,
        args.detectFiducials,
        !args.trackObject
      ));
    }
    else throw std::runtime_error("Unknown pipeline type: " + args.pipelineType);
  }
  else
  {
    // Set a reasonable default for the voxel size (this can be overridden using a configuration file).
    if(!settings->has_values("SceneParams.voxelSize"))
    {
      settings->sceneParams.voxelSize = 0.015f;
      settings->sceneParams.mu = settings->sceneParams.voxelSize * 4;
    }

    // Set up the image source engines, mapping modes, tracking modes and tracker configurations.
    std::vector<CompositeImageSourceEngine_Ptr> imageSourceEngines;
    std::vector<SLAMComponent::MappingMode> mappingModes;
    std::vector<SLAMComponent::TrackingMode> trackingModes;
    std::vector<std::string> trackerConfigs;

    // Add an image source engine for each disk sequence specified.
    for(size_t i = 0; i < args.sequences.size(); ++i)
    {
      const bf::path calibrationPath = args.sequences[i]->default_calib_path();
      const std::string calibrationFilename = bf::exists(calibrationPath) ? calibrationPath.string() : args.calibrationFilename;

      std::cout << "[spaint] Adding local agent for disk sequence: " << *args.sequences[i] << '\n';
      CompositeImageSourceEngine_Ptr imageSourceEngine(new CompositeImageSourceEngine);
      imageSourceEngine->addSubengine(new AsyncImageSourceEngine(args.sequences[i]->make_image_source_engine(calibrationFilename), args.prefetchBufferCapacity));

      imageSourceEngines.push_back(imageSourceEngine);
    }

    // Set up the mapping modes, tracking modes and tracker configurations.
    for(size_t i = 0, size = imageSourceEngines.size(); i < size; ++i)
    {
      mappingModes.push_back(args.mapSurfels ? SLAMComponent::MAP_BOTH : SLAMComponent::MAP_VOXELS_ONLY);
      trackingModes.push_back(args.trackSurfels ? SLAMComponent::TRACK_SURFELS : SLAMComponent::TRACK_VOXELS);

      // FIXME: We don't always want to read the poses from disk - make it possible to run the normal tracker instead.
      trackerConfigs.push_back(args.sequences[i]->make_disk_tracker_config());
    }

    // Construct the pipeline itself.
    const CollaborationMode collaborationMode = args.collaborationMode == "batch" ? CM_BATCH : CM_LIVE;
    pipeline.reset(new CollaborativePipeline(
      settings,
      Application::resources_dir().string(),
      imageSourceEngines,
      trackerConfigs,
      mappingModes,
      trackingModes,
      args.detectFiducials,
      mappingServer,
      collaborationMode
    ));
  }

  // If a remote host was specified, set up a mapping client for the world scene.
  if(args.host != "")
  {
    std::cout << "Setting mapping client for host '" << args.host << "' and port '" << args.port << "'\n";
    const pooled_queue::PoolEmptyStrategy poolEmptyStrategy = settings->get_first_value<pooled_queue::PoolEmptyStrategy>("MappingClient.poolEmptyStrategy", pooled_queue::PES_DISCARD);
    pipeline->set_mapping_client(Model::get_world_scene_id(), MappingClient_Ptr(new MappingClient(args.host, args.port, poolEmptyStrategy)));
  }

#ifdef WITH_LEAP
  // Set the ID of the fiducial to use for the Leap Motion (if any).
  pipeline->get_model()->set_leap_fiducial_id(args.leapFiducialID);
#endif

  // Configure and run the application.
  Application app(pipeline, args.renderFiducials);
  if(args.batch) app.set_batch_mode_enabled(true);
  if(args.runServer) app.set_server_mode_enabled(true);
  app.set_save_memory_usage(args.profileMemory);
  app.set_save_mesh_on_exit(args.saveMeshOnExit);
  app.set_save_models_on_exit(args.saveModelsOnExit);
  bool runSucceeded = app.run();

  // Close all open joysticks.
  joysticks.clear();

  // If we're not running in headless mode, shut down the GUI-only subsystems.
  if(!args.headless)
  {
  #ifdef WITH_OVR
    // If we built with Rift support, shut down the Rift SDK.
    ovr_Shutdown();
  #endif

    // Shut down SDL.
    SDL_Quit();
  }

  return runSucceeded ? EXIT_SUCCESS : EXIT_FAILURE;
}
catch(std::exception& e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
