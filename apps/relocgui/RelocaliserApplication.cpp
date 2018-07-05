/**
 * relocgui: RelocaliserApplication.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "RelocaliserApplication.h"

#include <iostream>
#include <stdexcept>

#include <boost/make_shared.hpp>

#include <Eigen/Geometry>

#include <grove/relocalisation/ScoreRelocaliserFactory.h>

#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Objects/Camera/ITMCalibIO.h>

#include <itmx/base/MemoryBlockFactory.h>
#include <itmx/persistence/PosePersister.h>

#include <tvgutil/filesystem/PathFinder.h>
#include <tvgutil/timing/AverageTimer.h>
#include <tvgutil/timing/TimeUtil.h>

namespace bf = boost::filesystem;

using namespace grove;
using namespace ITMLib;
using namespace itmx;
using namespace tvgutil;

namespace relocgui {

//#################### ANONYMOUS FREE FUNCTIONS ####################

namespace {

/**
 * \brief Computes the angular separation between two rotation matrices.
 *
 * \param r1 The first rotation matrix.
 * \param r2 The second rotation matrix.
 *
 * \return The angle of the transformation mapping r1 to r2.
 */
float angular_separation(const Eigen::Matrix3f &r1, const Eigen::Matrix3f &r2)
{
  // First calculate the rotation matrix which maps r1 to r2.
  Eigen::Matrix3f dr = r2 * r1.transpose();

  Eigen::AngleAxisf aa(dr);
  return aa.angle();
}

/**
 * \brief Check whether a pose matrix is similar enough to a ground truth pose matrix.
 *
 * \param gtPose               The ground truth pose matrix.
 * \param testPose             The candidate pose matrix.
 * \param translationMaxError  The maximum difference (in m) between the translation components of the two matrices.
 * \param angleMaxError        The maximum angular difference between the two rotation components (when converted to the
 *                             axis-angle representation).
 *
 * \return Whether or not the two poses differ for less or equal than translationMaxError and angleMaxError.
 */
bool pose_matches(const Matrix4f &gtPose, const Matrix4f &testPose, float translationMaxError, float angleMaxError)
{
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

/**
 * \brief Read a 4x4 pose matrix from a text file.
 *
 * \param fileName Path to the text file.
 *
 * \return The pose matrix.
 *
 * \throws std::runtime_error if the file is missing or has the wrong format.
 */
Matrix4f read_pose_from_file(const bf::path &fileName)
{
  if(!bf::is_regular(fileName)) throw std::runtime_error("File not found: " + fileName.string());

  std::ifstream in(fileName.string().c_str());

  Matrix4f res;
  in >> res(0, 0) >> res(1, 0) >> res(2, 0) >> res(3, 0);
  in >> res(0, 1) >> res(1, 1) >> res(2, 1) >> res(3, 1);
  in >> res(0, 2) >> res(1, 2) >> res(2, 2) >> res(3, 2);
  in >> res(0, 3) >> res(1, 3) >> res(2, 3) >> res(3, 3);

  if(!in) throw std::runtime_error("Error reading a pose matrix from: " + fileName.string());

  return res;
}
}

//#################### CONSTRUCTOR ####################

RelocaliserApplication::RelocaliserApplication(const std::string &calibrationPath,
                                               const std::string &trainingPath,
                                               const std::string &testingPath,
                                               const SettingsContainer_CPtr &settings)
  : m_calibrationFilePath(calibrationPath)
  , m_saveRelocalisedPoses(true)
  , m_settingsContainer(settings)
  , m_testSequencePath(testingPath)
  , m_trainSequencePath(trainingPath)
{
  // Read camera calibration parameters.
  if(!ITMLib::readRGBDCalib(m_calibrationFilePath.string().c_str(), m_cameraCalibration))
  {
    throw std::invalid_argument("Couldn't read calibration parameters.");
  }

  // Check that training and testing folders exist.
  if(!bf::is_directory(m_trainSequencePath))
  {
    throw std::invalid_argument("The specified training path does not exist.");
  }

  if(!bf::is_directory(m_testSequencePath))
  {
    throw std::invalid_argument("The specified testing path does not exist.");
  }

  // Setup the image masks.
  m_depthImageMask = m_settingsContainer->get_first_value<std::string>("depthImageMask", "frame-%06d.depth.png");
  m_poseFileMask = m_settingsContainer->get_first_value<std::string>("poseFileMask", "frame-%06d.pose.txt");
  m_rgbImageMask = m_settingsContainer->get_first_value<std::string>("rgbImageMask", "frame-%06d.color.png");

  // Create the folder that will store the relocalised poses.
  if(m_saveRelocalisedPoses)
  {
    const std::string experimentTag =
        m_settingsContainer->get_first_value<std::string>("experimentTag", TimeUtil::get_iso_timestamp());
    m_outputPosesPath = find_subdir_from_executable("reloc_poses") / experimentTag;

    std::cout << "Saving output poses in: " << m_outputPosesPath << '\n';
    if(!bf::is_directory(m_outputPosesPath))
    {
      bf::create_directories(m_outputPosesPath);
    }
  }

  // Set up the path generators.
  m_outputPosesPathGenerator = boost::make_shared<SequentialPathGenerator>(m_outputPosesPath);
  m_testingSequencePathGenerator = boost::make_shared<SequentialPathGenerator>(m_testSequencePath);
  m_trainingSequencePathGenerator = boost::make_shared<SequentialPathGenerator>(m_trainSequencePath);

  // We try to run everything on the GPU.
  const DeviceType deviceType = DEVICE_CUDA;

  // Setup the relocaliser.
  const bf::path resourcesFolder = find_subdir_from_executable("resources");
  const bf::path defaultRelocalisationForestPath = resourcesFolder / "DefaultRelocalisationForest.rf";
  std::cout << "Loading relocalisation forest from: " << defaultRelocalisationForestPath << '\n';

  m_relocaliser = ScoreRelocaliserFactory::make_score_relocaliser(defaultRelocalisationForestPath.string(), m_settingsContainer, deviceType);

  // Create a ViewBuilder to convert the depth image to float (might to it with OpenCV as well but this is easier).
  m_viewBuilder.reset(ITMViewBuilderFactory::MakeViewBuilder(m_cameraCalibration, deviceType));

  // Allocate the ITM Images used to train/test the relocaliser (empty for now, will be resized later).
  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_currentDepthImage = mbf.make_image<float>();
  m_currentRawDepthImage = mbf.make_image<short>();
  m_currentRgbImage = mbf.make_image<Vector4u>();
}

void RelocaliserApplication::run()
{
  boost::optional<RelocalisationExample> currentExample;

  std::cout << "Start training.\n";

  // First of all, train the relocaliser processing each image from the training folder.
  AverageTimer<boost::chrono::milliseconds> trainingTimer("Training Timer");
  while((currentExample = read_example(m_trainingSequencePathGenerator)))
  {
    trainingTimer.start();

    prepare_example_images(*currentExample);

    // Now train the relocaliser.
    m_relocaliser->train(m_currentRgbImage.get(),
                         m_currentDepthImage.get(),
                         m_cameraCalibration.intrinsics_d.projectionParamsSimple.all,
                         currentExample->cameraPose);

    // Finally, increment the index.
    m_trainingSequencePathGenerator->increment_index();

    // Stop the timer before the visualization calls.
    trainingTimer.stop();

    // Update UI
    show_example(*currentExample);
  }

  std::cout << "Training done, processed " << m_trainingSequencePathGenerator->get_index() << " RGB-D image pairs.\n";

  // Now test the relocaliser accumulating the number of successful relocalisations.
  uint32_t successfulExamples = 0;
  AverageTimer<boost::chrono::milliseconds> testingTimer("Testing Timer");
  while((currentExample = read_example(m_testingSequencePathGenerator)))
  {
    testingTimer.start();
    prepare_example_images(*currentExample);

    // Now relocalise.
    std::vector<Relocaliser::Result> relocaliserResults =
        m_relocaliser->relocalise(m_currentRgbImage.get(),
                                  m_currentDepthImage.get(),
                                  m_cameraCalibration.intrinsics_d.projectionParamsSimple.all);

    // This will store the relocalised pose (if we were successful).
    Matrix4f inverseCameraPose;
    inverseCameraPose.setValues(std::numeric_limits<float>::quiet_NaN());

    if(!relocaliserResults.empty())
    {
      inverseCameraPose = relocaliserResults[0].pose.GetInvM();
    }

    // Save the pose if required.
    if(m_saveRelocalisedPoses)
    {
      PosePersister::save_pose_on_thread(inverseCameraPose,
                                         m_outputPosesPathGenerator->make_path("pose-%06i.reloc.txt"));
    }

    // Check whether the relocalisation succeeded or not by looking at the ground truth pose (using the 7-scenes
    // 5cm/5deg criterion).
    static const float translationMaxError = 0.05f;
    static const float angleMaxError = 5.f * static_cast<float>(M_PI) / 180.f;
    const bool relocalisationSucceeded =
        pose_matches(currentExample->cameraPose.GetInvM(), inverseCameraPose, translationMaxError, angleMaxError);

    // Update stats.
    successfulExamples += relocalisationSucceeded;

    // Increment path generator indices.
    m_testingSequencePathGenerator->increment_index();
    m_outputPosesPathGenerator->increment_index();

    // Stop the timer before the visualization calls.
    testingTimer.stop();

    // Show the example and print whether the relocalisation succeeded or not.
    show_example(*currentExample, relocalisationSucceeded ? "Relocalisation OK" : "Relocalisation Failed");
  }

  const int testedExamples = m_testingSequencePathGenerator->get_index();
  std::cout << "Testing done.\n\nEvaluated " << testedExamples << " RGBD frames.\n";
  std::cout << successfulExamples
            << " frames were relocalised correctly (<=5cm translational and <=5deg angular error).\n";
  const float accuracy =
      100.f * (testedExamples > 0 ? static_cast<float>(successfulExamples) / static_cast<float>(testedExamples) : 0.0f);
  std::cout << "Overall accuracy: " << accuracy << "%\n";
  std::cout << trainingTimer << '\n';
  std::cout << testingTimer << '\n';
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

boost::optional<RelocaliserApplication::RelocalisationExample>
    RelocaliserApplication::read_example(const RelocaliserApplication::SequentialPathGenerator_Ptr &pathGenerator) const
{
  bf::path currentDepthPath = pathGenerator->make_path(m_depthImageMask);
  bf::path currentRgbPath = pathGenerator->make_path(m_rgbImageMask);
  bf::path currentPosePath = pathGenerator->make_path(m_poseFileMask);

  if(bf::is_regular(currentDepthPath) && bf::is_regular(currentRgbPath) && bf::is_regular(currentPosePath))
  {
    RelocalisationExample example;

    // Read the images.
    example.depthImage = cv::imread(currentDepthPath.string().c_str(), cv::IMREAD_ANYDEPTH);
    example.rgbImage = cv::imread(currentRgbPath.string().c_str());

    // The files store the inverse camera pose.
    example.cameraPose.SetInvM(read_pose_from_file(currentPosePath));

    // Convert from BGR to RGBA.
    cv::cvtColor(example.rgbImage, example.rgbImage, CV_BGR2RGBA);

    return example;
  }

  return boost::none;
}

void RelocaliserApplication::prepare_example_images(const RelocaliserApplication::RelocalisationExample &currentExample)
{
  const Vector2i depthImageDims(currentExample.depthImage.cols, currentExample.depthImage.rows);
  const Vector2i rgbImageDims(currentExample.rgbImage.cols, currentExample.rgbImage.rows);

  // Copy the Mats into the ITM images (the depth image needs conversion to float according to the calibration).

  // Resize them (usually NOOP).
  m_currentRawDepthImage->ChangeDims(depthImageDims);
  m_currentDepthImage->ChangeDims(depthImageDims);
  m_currentRgbImage->ChangeDims(rgbImageDims);

  // Perform copy using a Mat wrapper.
  currentExample.depthImage.convertTo(
      cv::Mat(depthImageDims.y, depthImageDims.x, CV_16SC1, m_currentRawDepthImage->GetData(MEMORYDEVICE_CPU)), CV_16S);
  currentExample.rgbImage.copyTo(
      cv::Mat(rgbImageDims.y, rgbImageDims.x, CV_8UC4, m_currentRgbImage->GetData(MEMORYDEVICE_CPU)));

  // Update them on the device.
  m_currentRawDepthImage->UpdateDeviceFromHost();
  m_currentRgbImage->UpdateDeviceFromHost();

  // Use the viewBuilder to prepare the depth image.
  m_viewBuilder->ConvertDepthAffineToFloat(
      m_currentDepthImage.get(), m_currentRawDepthImage.get(), m_cameraCalibration.disparityCalib.GetParams());
}

void RelocaliserApplication::show_example(const RelocalisationExample &example, const std::string &uiText) const
{
  static const std::string WINDOW_NAME = "Relocalisation GUI";

  // Setup a named window.
  cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);

  // Allocate a canvas big enough to hold the colour and depth image side by side.
  cv::Mat canvas = cv::Mat::zeros(std::max(example.depthImage.rows, example.rgbImage.rows),
                                  example.depthImage.cols + example.rgbImage.cols,
                                  CV_8UC3);

  // Copy the colour image to its location on the canvas (converting it into BGR format since that's what OpenCV uses
  // for visualization).
  cv::cvtColor(example.rgbImage, canvas(cv::Rect(0, 0, example.rgbImage.cols, example.rgbImage.rows)), CV_RGBA2BGR);

  // Normalize the depth image (black is very close, white is far away) and copy it to its location on the canvas.
  cv::Mat processedDepth;
  cv::normalize(example.depthImage, processedDepth, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::cvtColor(processedDepth,
               canvas(cv::Rect(example.rgbImage.cols, 0, example.depthImage.cols, example.depthImage.rows)),
               CV_GRAY2BGR);

  // Draw the text in the top-left corner, if required.
  if(!uiText.empty())
  {
    const double fontSize = 1.5;
    const int thickness = 2;

    // Compute the text's bounding box (we actually only care about its height).
    int baseLine = 0;
    cv::Size textSize = cv::getTextSize(uiText, cv::FONT_HERSHEY_SIMPLEX, fontSize, thickness, &baseLine);

    // Write the text on the image applying a "Poor man's shadow effect".
    cv::putText(canvas,
                uiText,
                cv::Point(12, 12 + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX,
                fontSize,
                cv::Scalar::all(0),
                thickness);
    cv::putText(canvas,
                uiText,
                cv::Point(10, 10 + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX,
                fontSize,
                cv::Scalar::all(255),
                thickness);
  }

  // Actualy show the image.
  cv::imshow(WINDOW_NAME, canvas);
  cv::waitKey(1);
}
}
