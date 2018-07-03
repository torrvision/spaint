/**
 * prepare_7scenes: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <fstream>
#include <iostream>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <tvgutil/filesystem/SequentialPathGenerator.h>
using namespace tvgutil;

//#################### NAMESPACE ALIASES ####################

namespace bf = boost::filesystem;

//#################### FUNCTIONS ####################

void copy_rgbd_pose_frame(const SequentialPathGenerator &inputPathGenerator,
                          const SequentialPathGenerator &outputPathGenerator,
                          const std::string &depthMask,
                          const std::string &rgbMask,
                          const std::string &poseMask)
{
  // The colour image can just be copied.
  bf::copy_file(inputPathGenerator.make_path(rgbMask), outputPathGenerator.make_path(rgbMask));

  // Same as the pose file.
  bf::copy_file(inputPathGenerator.make_path(poseMask), outputPathGenerator.make_path(poseMask));

  // The depth image needs extra care (need to set pixels with value == 65535 to 0).
  cv::Mat depthImage = cv::imread(inputPathGenerator.make_path(depthMask).string(), cv::IMREAD_ANYDEPTH);

  cv::Mat invalidMask = depthImage == 65535;
  depthImage.setTo(0, invalidMask);

  cv::imwrite(outputPathGenerator.make_path(depthMask).string(), depthImage);
}

void process_splits(const bf::path &sequencePath,
                    const std::vector<std::string> &splitNames,
                    const bf::path &outputPath,
                    int singleSplitLength,
                    const std::string &depthMask,
                    const std::string &rgbMask,
                    const std::string &poseMask)
{
  SequentialPathGenerator outputPathGenerator(outputPath);

  for(size_t splitIdx = 0; splitIdx < splitNames.size(); ++splitIdx)
  {
    const bf::path splitFolder = sequencePath / splitNames[splitIdx];
    SequentialPathGenerator splitPathGenerator(splitFolder);

    std::cout << "Processing images from: " << splitFolder << '\n';

    for(int i = 0; i < singleSplitLength; ++i)
    {
      copy_rgbd_pose_frame(splitPathGenerator, outputPathGenerator, depthMask, rgbMask, poseMask);

      // Increment indices.
      splitPathGenerator.increment_index();
      outputPathGenerator.increment_index();
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

//#################### MAIN ####################

int main(int argc, char *argv[]) try
{
  if(argc != 2)
  {
    quit(std::string("Usage: ") + argv[0] + " 7scenes_root.");
  }

  const bf::path datasetRoot = argv[1];
  if(!bf::is_directory(datasetRoot))
  {
    quit("The specified root folder does not exist.");
  }

  std::cout << "Preparing 7scenes dataset in: " << datasetRoot << '\n';

  // Filaname masks.
  const std::string depthFileMask = "frame-%06d.depth.png";
  const std::string poseFileMask = "frame-%06d.pose.txt";
  const std::string rgbFileMask = "frame-%06d.color.png";

  const std::string trainingFolderName = "train";
  const std::string testingFolderName = "test";

  // Dataset-specific informations.
  std::vector<std::string> sequenceNames = {"chess", "fire", "heads", "office", "pumpkin", "redkitchen", "stairs"};
  std::vector<int> sequenceSplitLengths = {1000, 1000, 1000, 1000, 1000, 1000, 500};

  std::map<std::string, std::vector<std::string> > trainingSplits;
  std::map<std::string, std::vector<std::string> > testingSplits;

  // Chess
  trainingSplits[sequenceNames[0]].push_back("seq-01");
  trainingSplits[sequenceNames[0]].push_back("seq-02");
  trainingSplits[sequenceNames[0]].push_back("seq-04");
  trainingSplits[sequenceNames[0]].push_back("seq-06");

  testingSplits[sequenceNames[0]].push_back("seq-03");
  testingSplits[sequenceNames[0]].push_back("seq-05");

  // Fire
  trainingSplits[sequenceNames[1]].push_back("seq-01");
  trainingSplits[sequenceNames[1]].push_back("seq-02");

  testingSplits[sequenceNames[1]].push_back("seq-03");
  testingSplits[sequenceNames[1]].push_back("seq-04");

  // Heads
  trainingSplits[sequenceNames[2]].push_back("seq-02");

  testingSplits[sequenceNames[2]].push_back("seq-01");

  // Office
  trainingSplits[sequenceNames[3]].push_back("seq-01");
  trainingSplits[sequenceNames[3]].push_back("seq-03");
  trainingSplits[sequenceNames[3]].push_back("seq-04");
  trainingSplits[sequenceNames[3]].push_back("seq-05");
  trainingSplits[sequenceNames[3]].push_back("seq-08");
  trainingSplits[sequenceNames[3]].push_back("seq-10");

  testingSplits[sequenceNames[3]].push_back("seq-02");
  testingSplits[sequenceNames[3]].push_back("seq-06");
  testingSplits[sequenceNames[3]].push_back("seq-07");
  testingSplits[sequenceNames[3]].push_back("seq-09");

  // Pumpkin
  trainingSplits[sequenceNames[4]].push_back("seq-02");
  trainingSplits[sequenceNames[4]].push_back("seq-03");
  trainingSplits[sequenceNames[4]].push_back("seq-06");
  trainingSplits[sequenceNames[4]].push_back("seq-08");

  testingSplits[sequenceNames[4]].push_back("seq-01");
  testingSplits[sequenceNames[4]].push_back("seq-07");

  // Redkitchen
  trainingSplits[sequenceNames[5]].push_back("seq-01");
  trainingSplits[sequenceNames[5]].push_back("seq-02");
  trainingSplits[sequenceNames[5]].push_back("seq-05");
  trainingSplits[sequenceNames[5]].push_back("seq-07");
  trainingSplits[sequenceNames[5]].push_back("seq-08");
  trainingSplits[sequenceNames[5]].push_back("seq-11");
  trainingSplits[sequenceNames[5]].push_back("seq-13");

  testingSplits[sequenceNames[5]].push_back("seq-03");
  testingSplits[sequenceNames[5]].push_back("seq-04");
  testingSplits[sequenceNames[5]].push_back("seq-06");
  testingSplits[sequenceNames[5]].push_back("seq-12");
  testingSplits[sequenceNames[5]].push_back("seq-14");

  // Stairs
  trainingSplits[sequenceNames[6]].push_back("seq-02");
  trainingSplits[sequenceNames[6]].push_back("seq-03");
  trainingSplits[sequenceNames[6]].push_back("seq-05");
  trainingSplits[sequenceNames[6]].push_back("seq-06");

  testingSplits[sequenceNames[6]].push_back("seq-01");
  testingSplits[sequenceNames[6]].push_back("seq-04");

  // First of all, check that every folder exists and create training and testing subfolders.
  for(size_t sequenceIdx = 0; sequenceIdx < sequenceNames.size(); ++sequenceIdx)
  {
    const std::string &sequenceName = sequenceNames[sequenceIdx];
    const bf::path sequenceRoot = datasetRoot / sequenceName;

    std::cout << "Checking folder structure for sequence " << sequenceName << " in: " << sequenceRoot << '\n';

    // Check for the root.
    if(!bf::is_directory(sequenceRoot))
    {
      quit(sequenceRoot.string() + " does not exist.");
    }

    // Check for the subfolders.
    const std::vector<std::string> &sequenceTrainingSplits = trainingSplits[sequenceName];
    for(size_t splitIdx = 0; splitIdx < sequenceTrainingSplits.size(); ++splitIdx)
    {
      const bf::path splitFolder = sequenceRoot / sequenceTrainingSplits[splitIdx];
      if(!bf::is_directory(splitFolder))
      {
        quit(splitFolder.string() + " does not exist.");
      }
    }

    const std::vector<std::string> &sequenceTestingSplits = testingSplits[sequenceName];
    for(size_t splitIdx = 0; splitIdx < sequenceTestingSplits.size(); ++splitIdx)
    {
      const bf::path splitFolder = sequenceRoot / sequenceTestingSplits[splitIdx];
      if(!bf::is_directory(splitFolder))
      {
        quit(splitFolder.string() + " does not exist.");
      }
    }

    // Create train and test subfolders.
    std::cout << "Creating train and test subfolders.\n";
    bf::create_directory(sequenceRoot / trainingFolderName);
    bf::create_directory(sequenceRoot / testingFolderName);
  }

  // Create calibration file.
  const std::string calibrationFile = "calib.txt";
  const bf::path calibrationFileName = datasetRoot / calibrationFile;

  {
    std::cout << "Creating calibration file: " << calibrationFileName << '\n';

    std::ofstream calibrationFile(calibrationFileName.string().c_str());
    calibrationFile << "640 480\n"
                    << "585 585\n"
                    << "320 240\n"
                    << '\n'
                    << "640 480\n"
                    << "585 585\n"
                    << "320 240\n"
                    << '\n'
                    << "1 0 0 0\n"
                    << "0 1 0 0\n"
                    << "0 0 1 0\n"
                    << '\n'
                    << "affine 0.001 0\n";
  }

  // Now prepare each sequence.
  for(size_t sequenceIdx = 0; sequenceIdx < sequenceNames.size(); ++sequenceIdx)
  {
    const std::string &sequenceName = sequenceNames[sequenceIdx];
    const bf::path sequenceRoot = datasetRoot / sequenceName;

    // Training splits.
    const bf::path trainingPath = sequenceRoot / trainingFolderName;
    std::cout << "Preparing training sequence: " << trainingPath << '\n';
    bf::copy_file(calibrationFileName, trainingPath / calibrationFile);
    process_splits(sequenceRoot,
                   trainingSplits[sequenceName],
                   trainingPath,
                   sequenceSplitLengths[sequenceIdx],
                   depthFileMask,
                   rgbFileMask,
                   poseFileMask);

    // Testing splits.
    const bf::path testingPath = sequenceRoot / testingFolderName;
    std::cout << "Preparing testing sequence: " << testingPath << '\n';
    bf::copy_file(calibrationFileName, testingPath / calibrationFile);
    process_splits(sequenceRoot,
                   testingSplits[sequenceName],
                   testingPath,
                   sequenceSplitLengths[sequenceIdx],
                   depthFileMask,
                   rgbFileMask,
                   poseFileMask);
  }

  return EXIT_SUCCESS;
}
catch(std::exception &e)
{
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}
