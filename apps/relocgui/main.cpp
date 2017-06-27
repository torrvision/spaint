/**
 * relocgui: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Objects/Camera/ITMCalibIO.h>
#include <ITMLib/Objects/Camera/ITMRGBDCalib.h>

#include <grove/relocalisation/ScoreRelocaliserFactory.h>

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMObjectPtrTypes.h>
#include <itmx/base/MemoryBlockFactory.h>
#include <itmx/persistence/PosePersister.h>

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/filesystem/SequentialPathGenerator.h>
#include <tvgutil/misc/SettingsContainer.h>
#include <tvgutil/timing/TimeUtil.h>

using namespace ITMLib;

using namespace grove;
using namespace itmx;
using namespace tvgutil;

//#################### NAMESPACE ALIASES ####################

namespace bf = boost::filesystem;
namespace po = boost::program_options;

static const std::string WINDOW_NAME = "Relocalisation GUI";

//#################### TYPES ####################

struct CommandLineArguments
{
  //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~

  // User-specifiable arguments
  std::string calibrationFilename;
  std::string depthImageMask;
  std::string experimentTag;
  std::string poseFileMask;
  std::string rgbImageMask;
  std::string testFolder;
  std::string trainFolder;

  //~~~~~~~~~~~~~~~~~~~~ PUBLIC MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~

  /**
   * \brief Adds the command-line arguments to a settings object.
   *
   * \param settings  The settings object.
   */
  void add_to_settings(const SettingsContainer_Ptr &settings)
  {
#define ADD_SETTING(arg) settings->add_value(#arg, boost::lexical_cast<std::string>(arg))
#define ADD_SETTINGS(arg)                                                                                              \
  for(size_t i = 0; i < arg.size(); ++i)                                                                               \
  {                                                                                                                    \
    settings->add_value(#arg, boost::lexical_cast<std::string>(arg[i]));                                               \
  }
    ADD_SETTING(calibrationFilename);
    ADD_SETTING(depthImageMask);
    ADD_SETTING(experimentTag);
    ADD_SETTING(poseFileMask);
    ADD_SETTING(rgbImageMask);
    ADD_SETTING(testFolder);
    ADD_SETTING(trainFolder);
#undef ADD_SETTINGS
#undef ADD_SETTING
  }
};

struct RelocalisationExample
{
  cv::Mat depthImage;
  cv::Mat rgbImage;
  ORUtils::SE3Pose cameraPose;
};

//#################### FUNCTIONS ####################

void show_example(const RelocalisationExample &example, const std::string &text = "")
{
  cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);

  cv::Mat canvas = cv::Mat::zeros(std::max(example.depthImage.rows, example.rgbImage.rows),
                                  example.depthImage.cols + example.rgbImage.cols,
                                  CV_8UC3);

  cv::cvtColor(example.rgbImage, canvas(cv::Rect(0, 0, example.rgbImage.cols, example.rgbImage.rows)), CV_RGBA2BGR);

  cv::Mat processedDepth;
  cv::normalize(example.depthImage, processedDepth, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::cvtColor(processedDepth,
               canvas(cv::Rect(example.rgbImage.cols, 0, example.depthImage.cols, example.depthImage.rows)),
               CV_GRAY2BGR);

  if(!text.empty())
  {
    const double fontSize = 1.5;
    const int thickness = 2;
    int baseLine = 0;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fontSize, thickness, &baseLine);

    // Poor man's shadow.
    cv::putText(canvas,
                text,
                cv::Point(12, 12 + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX,
                fontSize,
                cv::Scalar::all(0),
                thickness);
    cv::putText(canvas,
                text,
                cv::Point(10, 10 + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX,
                fontSize,
                cv::Scalar::all(255),
                thickness);
  }

  cv::imshow(WINDOW_NAME, canvas);
  cv::waitKey(1);
}

float angular_separation(const Eigen::Matrix3f &r1, const Eigen::Matrix3f &r2)
{
  // First calculate the rotation matrix which maps r1 to r2.
  Eigen::Matrix3f dr = r2 * r1.transpose();

  Eigen::AngleAxisf aa(dr);
  return aa.angle();
}

bool pose_matches(const Matrix4f &gtPose, const Matrix4f &testPose)
{
  static const float translationMaxError = 0.05f;
  static const float angleMaxError = 5.f * M_PI / 180.f;

  // Both our Matrix type and Eigen's are column major, so we can just use Map here.
  const Eigen::Map<const Eigen::Matrix4f> gtPoseEigen(gtPose.m);
  const Eigen::Map<const Eigen::Matrix4f> testPoseEigen(testPose.m);

  const Eigen::Matrix3f gtR = gtPoseEigen.block<3, 3>(0, 0);
  const Eigen::Matrix3f testR = testPoseEigen.block<3, 3>(0, 0);
  const Eigen::Vector3f gtT = gtPoseEigen.block<3, 1>(0, 3);
  const Eigen::Vector3f testT = testPoseEigen.block<3, 1>(0, 3);

  const float translationError = (gtT - testT).norm();
  const float angleError = angular_separation(gtR, testR);

  return translationError <= translationMaxError && angleError <= angleMaxError;
}

Matrix4f read_pose_from_file(const bf::path &fileName)
{
  if(!bf::is_regular(fileName)) throw std::runtime_error("File not found: " + fileName.string());

  std::ifstream in(fileName.c_str());

  Matrix4f res;
  in >> res(0, 0) >> res(1, 0) >> res(2, 0) >> res(3, 0);
  in >> res(0, 1) >> res(1, 1) >> res(2, 1) >> res(3, 1);
  in >> res(0, 2) >> res(1, 2) >> res(2, 2) >> res(3, 2);
  in >> res(0, 3) >> res(1, 3) >> res(2, 3) >> res(3, 3);

  return res;
}

boost::optional<RelocalisationExample> read_example(const std::string &depthMask,
                                                    const std::string &rgbMask,
                                                    const std::string &poseMask,
                                                    const SequentialPathGenerator &pathGenerator)
{
  bf::path currentDepthPath = pathGenerator.make_path(depthMask);
  bf::path currentRgbPath = pathGenerator.make_path(rgbMask);
  bf::path currentPosePath = pathGenerator.make_path(poseMask);

  if(bf::is_regular(currentDepthPath) && bf::is_regular(currentRgbPath) && bf::is_regular(currentPosePath))
  {
    RelocalisationExample example;
    example.depthImage = cv::imread(currentDepthPath.c_str(), cv::IMREAD_ANYDEPTH);
    example.rgbImage = cv::imread(currentRgbPath.c_str());
    example.cameraPose.SetInvM(read_pose_from_file(currentPosePath));

    // Convert BGR to RGBA.
    cv::cvtColor(example.rgbImage, example.rgbImage, CV_BGR2RGBA);

    return example;
  }

  return boost::none;
}

/**
 * \brief Adds any unregistered options in a set of parsed options to a settings container object.
 *
 * \param parsedOptions The set of parsed options.
 * \param settings      The settings container object.
 */
void add_unregistered_options_to_settings(const po::parsed_options &parsedOptions,
                                          const SettingsContainer_Ptr &settings)
{
  for(size_t i = 0, optionCount = parsedOptions.options.size(); i < optionCount; ++i)
  {
    const po::basic_option<char> &option = parsedOptions.options[i];
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
 * \brief Outputs the specified error message and terminates the program with the specified exit code.
 *
 * \param message The error message.
 * \param code    The exit code.
 */
void quit(const std::string &message, int code = EXIT_FAILURE)
{
  std::cerr << message << '\n';
  exit(code);
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
bool parse_command_line(int argc, char *argv[], CommandLineArguments &args, const SettingsContainer_Ptr &settings)
{
  // Specify the possible options.
  po::options_description genericOptions("Generic options");
  genericOptions.add_options()("help", "produce help message")(
      "configFile,f", po::value<std::string>(), "additional parameters filename")(
      "experimentTag", po::value<std::string>(&args.experimentTag)->default_value(""), "experiment tag");

  po::options_description diskSequenceOptions("Disk sequence options");
  diskSequenceOptions.add_options()(
      "calib,c", po::value<std::string>(&args.calibrationFilename)->required(), "calibration filename")(
      "depthMask",
      po::value<std::string>(&args.depthImageMask)->default_value("frame-%06d.depth.png"),
      "depth image mask")("test",
                          po::value<std::string>(&args.testFolder)->required(),
                          "path to the folder containing the training sequence")(
      "train",
      po::value<std::string>(&args.trainFolder)->required(),
      "path to the folder containing the training sequence")(
      "poseMask", po::value<std::string>(&args.poseFileMask)->default_value("frame-%06d.pose.txt"), "pose file mask")(
      "rgbMask", po::value<std::string>(&args.rgbImageMask)->default_value("frame-%06d.color.png"), "RGB image mask");

  po::options_description options;
  options.add(genericOptions);
  options.add(diskSequenceOptions);

  // Parse the command line.
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, options), vm);

  // If a configuration file was specified:
  if(vm.count("configFile"))
  {
    // Parse additional options from the configuration file and add any registered options to the variables map.
    // These will be post-processed (if necessary) and added to the settings later. Unregistered options are
    // also allowed: we add these directly to the settings without post-processing.
    po::parsed_options parsedConfigFileOptions =
        po::parse_config_file<char>(vm["configFile"].as<std::string>().c_str(), options, true);

    // Store registered options in the variables map.
    po::store(parsedConfigFileOptions, vm);

    // Add any unregistered options directly to the settings.
    add_unregistered_options_to_settings(parsedConfigFileOptions, settings);
  }

  po::notify(vm);

  // Add the registered options to the settings.
  args.add_to_settings(settings);

  std::cout << "Global settings:\n" << *settings << '\n';

  // If the user specifies the --help flag, print a help message.
  if(vm.count("help"))
  {
    std::cout << options << '\n';
    return false;
  }

  return true;
}

int main(int argc, char *argv[]) try
{
  // Construct the settings object for the application.
  SettingsContainer_Ptr settings(new SettingsContainer);

  // Parse the command-line arguments.
  CommandLineArguments args;
  if(!parse_command_line(argc, argv, args, settings))
  {
    return 0;
  }

  // Pass the device type to the memory block factory.
  const ITMLibSettings::DeviceType deviceType = ITMLibSettings::DEVICE_CUDA;
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  mbf.set_device_type(deviceType);

  // Read camera calibration parameters.
  ITMRGBDCalib cameraCalibration;
  if(!ITMLib::readRGBDCalib(args.calibrationFilename.c_str(), cameraCalibration))
  {
    quit("Couldn't read calibration parameters.");
  }

  // Useful paths.
  const bf::path resourcesFolder = find_subdir_from_executable("resources");
  const bf::path trainingSequencePath(args.trainFolder);
  const bf::path testingSequencePath(args.testFolder);
  const bf::path outputPosesPath = find_subdir_from_executable("reloc_poses") /
                                   (args.experimentTag.empty() ? TimeUtil::get_iso_timestamp() : args.experimentTag);

  // Check that training and testing folder exist.
  if(!bf::is_directory(trainingSequencePath))
  {
    quit("The specified training sequence is not a folder.");
  }

  if(!bf::is_directory(testingSequencePath))
  {
    quit("The specified testing sequence is not a folder.");
  }

  // Setup the output folder.
  std::cout << "Saving output poses in: " << outputPosesPath << '\n';
  if(!bf::is_directory(outputPosesPath))
  {
    bf::create_directories(outputPosesPath);
  }

  // Setup the relocaliser.
  const bf::path defaultRelocalisationForestPath = resourcesFolder / "DefaultRelocalisationForest.rf";
  std::cout << "Loading relocalisation forest from: " << defaultRelocalisationForestPath << '\n';

  Relocaliser_Ptr relocaliser =
      ScoreRelocaliserFactory::make_score_relocaliser(deviceType, settings, defaultRelocalisationForestPath.string());

  SequentialPathGenerator trainingSequencePathGenerator(trainingSequencePath);
  SequentialPathGenerator testingSequencePathGenerator(testingSequencePath);
  SequentialPathGenerator outputPosesPathGenerator(outputPosesPath);

  // Create a ViewBuilder to convert the depth image to float (might to it with OpenCV as well but this is easier).
  ViewBuilder_Ptr viewBuilder(ITMViewBuilderFactory::MakeViewBuilder(cameraCalibration, deviceType));

  // Allocate the ITM Images used to train/test the relocaliser (empty for now, will be resized later).
  ITMShortImage_Ptr currentRawDepthImage = mbf.make_image<short>();
  ITMFloatImage_Ptr currentDepthImage = mbf.make_image<float>();
  ITMUChar4Image_Ptr currentRgbImage = mbf.make_image<Vector4u>();

  boost::optional<RelocalisationExample> currentExample;

  // Train the relocaliser.
  while((currentExample =
             read_example(args.depthImageMask, args.rgbImageMask, args.poseFileMask, trainingSequencePathGenerator)))
  {
    const Vector2i imageDims(currentExample->depthImage.cols, currentExample->depthImage.rows);

    // Copy the Mats into the ITM images (the depth image needs conversion to float according to the calibration).

    // Resize them (usually NOOP).
    currentRawDepthImage->ChangeDims(imageDims);
    currentDepthImage->ChangeDims(imageDims);
    currentRgbImage->ChangeDims(imageDims);

    // Perform copy using a Mat wrapper.
    currentExample->depthImage.convertTo(
        cv::Mat(imageDims.y, imageDims.x, CV_16SC1, currentRawDepthImage->GetData(MEMORYDEVICE_CPU)), CV_16S);
    currentExample->rgbImage.copyTo(
        cv::Mat(imageDims.y, imageDims.x, CV_8UC4, currentRgbImage->GetData(MEMORYDEVICE_CPU)));

    // Update them on the device.
    currentRawDepthImage->UpdateDeviceFromHost();
    currentRgbImage->UpdateDeviceFromHost();

    // Use the viewBuilder to prepare the depth image.
    viewBuilder->ConvertDepthAffineToFloat(
        currentDepthImage.get(), currentRawDepthImage.get(), cameraCalibration.disparityCalib.GetParams());

    // Now train the relocaliser.
    relocaliser->train(currentRgbImage.get(),
                       currentDepthImage.get(),
                       cameraCalibration.intrinsics_d.projectionParamsSimple.all,
                       currentExample->cameraPose);

    show_example(*currentExample);

    trainingSequencePathGenerator.increment_index();
  }

  std::cout << "Training done.\n";

  // Now test the relocaliser.
  uint32_t testedExamples = 0;
  uint32_t successfulExamples = 0;

  while((currentExample =
             read_example(args.depthImageMask, args.rgbImageMask, args.poseFileMask, testingSequencePathGenerator)))
  {
    const Vector2i imageDims(currentExample->depthImage.cols, currentExample->depthImage.rows);

    // Copy the Mats into the ITM images (the depth image needs conversion to float according to the calibration).

    // Resize them (usually NOOP).
    currentRawDepthImage->ChangeDims(imageDims);
    currentDepthImage->ChangeDims(imageDims);
    currentRgbImage->ChangeDims(imageDims);

    // Perform copy using a Mat wrapper.
    currentExample->depthImage.convertTo(
        cv::Mat(imageDims.y, imageDims.x, CV_16SC1, currentRawDepthImage->GetData(MEMORYDEVICE_CPU)), CV_16S);
    currentExample->rgbImage.copyTo(
        cv::Mat(imageDims.y, imageDims.x, CV_8UC4, currentRgbImage->GetData(MEMORYDEVICE_CPU)));

    // Update them on the device.
    currentRawDepthImage->UpdateDeviceFromHost();
    currentRgbImage->UpdateDeviceFromHost();

    // Use the viewBuilder to prepare the depth image.
    viewBuilder->ConvertDepthAffineToFloat(
        currentDepthImage.get(), currentRawDepthImage.get(), cameraCalibration.disparityCalib.GetParams());

    // Now relocalise.
    boost::optional<Relocaliser::Result> relocaliserResult = relocaliser->relocalise(
        currentRgbImage.get(), currentDepthImage.get(), cameraCalibration.intrinsics_d.projectionParamsSimple.all);

    Matrix4f inverseCameraPose;
    inverseCameraPose.setValues(std::numeric_limits<float>::quiet_NaN());

    if(relocaliserResult)
    {
      inverseCameraPose = relocaliserResult->pose.GetInvM();
    }

    // Save the pose.
    PosePersister::save_pose_on_thread(inverseCameraPose, outputPosesPathGenerator.make_path("pose-%06i.reloc.txt"));
    PosePersister::save_pose_on_thread(inverseCameraPose, outputPosesPathGenerator.make_path("pose-%06i.icp.txt"));

    // Show the example and print whether the relocalisation succeeded or not.
    bool relocalisationSucceeded = pose_matches(currentExample->cameraPose.GetInvM(), inverseCameraPose);
    show_example(*currentExample, relocalisationSucceeded ? "Relocalisation OK" : "Relocalisation Failed");

    testedExamples++;
    successfulExamples += relocalisationSucceeded;

    testingSequencePathGenerator.increment_index();
    outputPosesPathGenerator.increment_index();
  }

  std::cout << "Testing done.\n\nEvaluated " << testedExamples << " RGBD frames.\n";
  std::cout << successfulExamples << " were relocalised correctly (<5cm and <5deg error).\n";
  const float accuracy =
      100.f * testedExamples > 0 ? static_cast<float>(successfulExamples) / static_cast<float>(testedExamples) : 0.0f;
  std::cout << "Overall accuracy: " << accuracy << '\n';

  return EXIT_SUCCESS;
}
catch(std::exception &e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
