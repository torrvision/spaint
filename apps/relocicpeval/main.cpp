/**
 * relocicpeval: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Geometry>

#include <tvgutil/filesystem/SequentialPathGenerator.h>
using namespace tvgutil;

//#################### NAMESPACE ALIASES ####################

namespace fs = boost::filesystem;
namespace po = boost::program_options;

//#################### CONSTANTS ####################

static const std::string trainFolderName = "train";
static const std::string validationFolderName = "validation";
static const std::string testFolderName = "test";

//#################### FUNCTIONS ####################

/**
 * \brief Finds the dataset sequences under a root folder.
 *
 *        The assumption is that each valid sequence folder will have both
 *        "train" and "test" subfolders.
 *
 * \param dataset_path The path to a dataset.
 *
 * \return A list of sequence names.
 */
std::vector<std::string> find_sequence_names(const fs::path &dataset_path)
{
  std::vector<std::string> sequences;

  // Iterate over every subfolder of the dataset.
  for(fs::directory_iterator it(dataset_path), end; it != end; ++it)
  {
    fs::path p = it->path();
    fs::path train_path = p / trainFolderName;
    fs::path test_path = p / testFolderName;

    // If the current folder has both a train and test subfolder, we store its name as a valid sequence.
    if(fs::is_directory(train_path) && fs::is_directory(test_path))
    {
      sequences.push_back(p.filename().string());
    }
  }

  // Sort sequence names because the directory iterator does not ensure ordering
  std::sort(sequences.begin(), sequences.end());

  return sequences;
}

/**
 * \brief Reads a rigid pose from disk. The pose has to be stored as a 4x4 row-major matrix.
 *
 * \param fileName The filename.
 *
 * \return The rigid transformation.
 *
 * \throws std::runtime_error if the file is missing.
 */
Eigen::Matrix4f read_pose_from_file(const fs::path &fileName)
{
  if(!fs::is_regular(fileName)) throw std::runtime_error("File not found: " + fileName.string());

  std::ifstream in(fileName.c_str());

  // The Matrix is stored in row-major order on disk.
  Eigen::Matrix4f res;
  in >> res(0, 0) >> res(0, 1) >> res(0, 2) >> res(0, 3);
  in >> res(1, 0) >> res(1, 1) >> res(1, 2) >> res(1, 3);
  in >> res(2, 0) >> res(2, 1) >> res(2, 2) >> res(2, 3);
  in >> res(3, 0) >> res(3, 1) >> res(3, 2) >> res(3, 3);

  // If parsing the file failed for any reason, set the pose to NaNs.
  if(!in)
  {
    res.setConstant(std::numeric_limits<float>::quiet_NaN());
  }

  return res;
}

/**
 * \brief Computes the angular separation between two rotation matrices.
 *
 * \param r1 The first rotation matrix.
 * \param r2 The second rotation matrix.
 *
 * \return The angular difference between the two transformations.
 */
float angular_separation(const Eigen::Matrix3f &r1, const Eigen::Matrix3f &r2)
{
  // First calculate the rotation matrix which maps r1 to r2.
  Eigen::Matrix3f dr = r2 * r1.transpose();

  // Then compute the corresponding angle-axis transform and return the angle.
  Eigen::AngleAxisf aa(dr);
  return aa.angle();
}

/**
 * \brief Compute the difference between two poses.
 *
 * \param poseA           The ground truth pose.
 * \param poseB           The pose to be tested.
 * \param translationDiff Returns the translational difference.
 * \param angleDiff       Returns the angular difference.
 */
void pose_difference(const Eigen::Matrix4f &poseA,
                     const Eigen::Matrix4f &poseB,
                     float &translationDiff,
                     float &angleDiff)
{
  const Eigen::Matrix3f aR = poseA.block<3, 3>(0, 0);
  const Eigen::Matrix3f bR = poseB.block<3, 3>(0, 0);
  const Eigen::Vector3f aT = poseA.block<3, 1>(0, 3);
  const Eigen::Vector3f bT = poseB.block<3, 1>(0, 3);

  // Compute the difference between the transformations.
  translationDiff = (aT - bT).norm();
  angleDiff = angular_separation(aR, bR);
}

/**
 * \brief Check whether the two poses are similar enough.
 *
 *        The check is performed according to the 7-scenes metric: succeeds if the translation between the
 *        transformations is <= 5cm and the angle is <= 5 deg.
 *
 * \param gtPose           The ground truth pose.
 * \param testPose         The pose to be tested.
 * \param translationError Returns the translational error.
 * \param angleError       Returns the angular error.
 *
 * \return Whether the two poses are similar enough.
 */
bool pose_matches(const Eigen::Matrix4f &gtPose,
                  const Eigen::Matrix4f &testPose,
                  float &translationError,
                  float &angleError)
{
  // 7-scenes thresholds.
  static const float translationMaxError = 0.05f;
  static const float angleMaxError = 5.f * M_PI / 180.f;

  // Compute the difference between the transformations.
  pose_difference(gtPose, testPose, translationError, angleError);

  return translationError <= translationMaxError && angleError <= angleMaxError;
}

/**
 * \brief Check whether the two poses are similar enough.
 *
 *        The check is performed according to the 7-scenes metric: succeeds if the translation between the
 *        transformations is <= 5cm and the angle is <= 5 deg.
 *
 * \param gtPose   The ground truth pose.
 * \param testPose The pose to be tested.
 *
 * \return Whether the two poses are similar enough.
 */
bool pose_matches(const Eigen::Matrix4f &gtPose, const Eigen::Matrix4f &testPose)
{
  float angleError, translationError;

  return pose_matches(gtPose, testPose, translationError, angleError);
}

/**
 * \brief Check whether a pose stored in a text file matches with a ground truth pose, according to the 7-scenes metric.
 *
 * \param gtPose   The ground truth camera pose.
 * \param poseFile The path to a file storing a transformation matrix.
 *
 * \return Whether the pose stored in the file matches the ground truth pose. False if the file is missing.
 */
bool pose_file_matches(const Eigen::Matrix4f &gtPose, const fs::path &poseFile)
{
  if(!fs::is_regular(poseFile)) return false;

  const Eigen::Matrix4f otherPose = read_pose_from_file(poseFile);
  return pose_matches(gtPose, otherPose);
}

/**
 * \brief Check whether a pose stored in a text file matches with a ground truth pose, according to the 7-scenes metric.
 *
 * \param gtPose   The ground truth camera pose.
 * \param poseFile The path to a file storing a transformation matrix.
 * \param translationError Returns the translational error (Infinity if the file does not exist).
 * \param angleError       Returns the angular error (Infinity if the file does not exist).
 *
 * \return Whether the pose stored in the file matches the ground truth pose. False if the file is missing.
 */
bool pose_file_matches(const Eigen::Matrix4f &gtPose,
                       const fs::path &poseFile,
                       float &translationError,
                       float &angleError)
{
  translationError = std::numeric_limits<float>::infinity();
  angleError = std::numeric_limits<float>::infinity();

  if(!fs::is_regular(poseFile)) return false;

  const Eigen::Matrix4f otherPose = read_pose_from_file(poseFile);
  return pose_matches(gtPose, otherPose, translationError, angleError);
}

struct PosePairResult
{
  float relocErrorA;
  float relocErrorT;
  bool relocSuccess;

  float icpErrorA;
  float icpErrorT;
  bool icpSuccess;

  float refinementDeltaA;
  float refinementDeltaT;
};

typedef std::vector<PosePairResult> SequenceResults;

/**
 * \brief Process a dataset sequence computing how well the refinement performed on it.
 *
 * \param gtFolder    The folder storing the ground truth poses.
 * \param relocFolder The folder storing the relocalisation results.
 *
 * \return The vector of PosePairResult on this sequence.
 */
SequenceResults evaluate_sequence(const fs::path &gtFolder, const fs::path &relocFolder)
{
  SequenceResults res;

  // Create appropriate path generators.
  SequentialPathGenerator gtPathGenerator(gtFolder);
  SequentialPathGenerator relocPathGenerator(relocFolder);

  while(true)
  {
    // Generate the paths to evaluate.
    const fs::path gtPath = gtPathGenerator.make_path("frame-%06i.pose.txt");
    const fs::path relocPath = relocPathGenerator.make_path("pose-%06i.reloc.txt");
    const fs::path icpPath = relocPathGenerator.make_path("pose-%06i.icp.txt");

    // If the GT file is missing this sequence is over.
    if(!fs::is_regular(gtPath)) break;

    // Read the GT camera pose.
    const Eigen::Matrix4f gtPose = read_pose_from_file(gtPath);
    const Eigen::Matrix4f relocPose = read_pose_from_file(relocPath);
    const Eigen::Matrix4f icpPose = read_pose_from_file(icpPath);

    float relocalisationTranslationError, relocalisationAngleError;
    float icpTranslationError, icpAngleError;
    float refinementDeltaT, refinementDeltaA;

    // Check whether different kinds of relocalisations succeeded.
    bool validReloc = pose_matches(gtPose, relocPose, relocalisationTranslationError, relocalisationAngleError);
    bool validICP = pose_matches(gtPose, icpPose, icpTranslationError, icpAngleError);
    pose_difference(relocPose, icpPose, refinementDeltaT, refinementDeltaA);

    PosePairResult pairResult;

    pairResult.relocErrorA = relocalisationAngleError * 180.0f / M_PI;
    pairResult.relocErrorT = relocalisationTranslationError;
    pairResult.relocSuccess = validReloc;

    pairResult.icpErrorA = icpAngleError * 180.0f / M_PI;
    pairResult.icpErrorT = icpTranslationError;
    pairResult.icpSuccess = validICP;

    pairResult.refinementDeltaA = refinementDeltaA * 180.0f / M_PI;
    pairResult.refinementDeltaT = refinementDeltaT;

    res.push_back(pairResult);

    // Increment counters.
    gtPathGenerator.increment_index();
    relocPathGenerator.increment_index();
  }

  return res;
}

/**
 * \brief Print a variable allocating to it a certain width on screen.
 */
template <typename T>
void printWidth(const T &item, int width, bool leftAlign = false)
{
  std::cerr << (leftAlign ? std::left : std::right) << std::setw(width) << std::fixed << std::setprecision(3) << item;
}

int main(int argc, char *argv[])
{
  fs::path datasetFolder;
  fs::path relocBaseFolder;
  std::string relocTag;
  bool useValidation = true;

  // Declare some options for the evaluation.
  po::options_description options("Relocperf Options");
  options.add_options()
      ("datasetFolder,d", po::value(&datasetFolder)->required(), "The path to the dataset.")
      ("relocBaseFolder,r", po::value(&relocBaseFolder)->required(), "The path to the folder where the relocalised poses are stored.")
      ("relocTag,t", po::value(&relocTag)->required(), "The tag assigned to the experiment to evaluate.")
      ("help,h", "Print this help message.")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, options), vm);

  if(vm.count("help"))
  {
    std::cerr << "Usage: " << options << '\n';
    exit(0);
  }

  try
  {
    po::notify(vm);
  }
  catch(const po::error &e)
  {
    std::cerr << "Error parsing the options: " << e.what() << '\n';
    std::cerr << options << '\n';
    exit(1);
  }

  // Find the valid sequences in the dataset folder.
  const std::vector<std::string> sequenceNames = find_sequence_names(datasetFolder);

  std::map<std::string, SequenceResults> results;

  // Evaluate each sequence.
  for(size_t sequenceIdx = 0; sequenceIdx < sequenceNames.size(); ++sequenceIdx)
  {
    const std::string &sequence = sequenceNames[sequenceIdx];

    // Compute the full paths.
    const fs::path gtPath = datasetFolder / sequence / (useValidation ? validationFolderName : testFolderName);
    const fs::path relocFolder = relocBaseFolder / (relocTag + '_' + sequence);

    std::cerr << "Processing sequence " << sequence << " in: " << gtPath << "\t - " << relocFolder << std::endl;
    try
    {
      results[sequence] = evaluate_sequence(gtPath, relocFolder);
    }
    catch(std::runtime_error &)
    {
      std::cerr << "\tSequence has not been evaluated.\n";
    }
  }

  // Prepare the CSV file.
  std::string outputFile = relocTag + "_RefinementEval.csv";
  std::cout << "Saving refinement stats to: " << outputFile << "\n";
  std::ofstream out(outputFile.c_str());

  out << "Sequence,FrameIdx,RelocErrorA,RelocErrorT,RelocSuccess,ICPErrorA,ICPErrorT,ICPSuccess,RefinementDeltaA,RefinementDeltaT\n";

  // Output everything.
  for(size_t sequenceIdx = 0; sequenceIdx < sequenceNames.size(); ++sequenceIdx)
  {
    const std::string &sequence = sequenceNames[sequenceIdx];
    const SequenceResults &sequenceResults = results[sequence];

    for(size_t i = 0; i < sequenceResults.size(); ++i)
    {
        const PosePairResult& x = sequenceResults[i];

        out << sequence << "," << i << "," << x.relocErrorA << "," << x.relocErrorT << "," << x.relocSuccess << "," << x.icpErrorA << "," << x.icpErrorT << "," << x.icpSuccess << "," << x.refinementDeltaA << "," << x.refinementDeltaT << "\n";
    }
  }

  return 0;
}
