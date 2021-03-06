/**
 * relocperf: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

#include <boost/algorithm/string.hpp>
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
 * \brief Finds the names of all of the valid sequences within the specified dataset.
 *
 * \note  We define a "valid" sequence to be one whose directory contains both "train" and "test" subdirectories.
 *
 * \param datasetRoot The root directory of the dataset.
 * \return            The names of all of the valid sequences within the dataset, in lexicographic order.
 */
std::vector<std::string> find_sequence_names(const fs::path& datasetRoot)
{
  std::vector<std::string> sequences;

  // For each subdirectory of the dataset root:
  for(fs::directory_iterator it(datasetRoot), end; it != end; ++it)
  {
    // Check to see whether it contains "train" and "test" subdirectories, and add it to the list of valid sequences if so.
    fs::path p = it->path();
    fs::path trainPath = p / trainFolderName;
    fs::path testPath = p / testFolderName;

    if(fs::is_directory(trainPath) && fs::is_directory(testPath))
    {
      sequences.push_back(p.filename().string());
    }
  }

  // Finally, sort the list of sequences names (since the directory iterator does not guarantee any particular ordering).
  std::sort(sequences.begin(), sequences.end());

  return sequences;
}

/**
 * \brief Reads a 4x4 matrix from a file on disk (in row-major order).
 *
 * \param filename  The name of the file containing the matrix.
 * \return          The matrix.
 *
 * \throws std::runtime_error If the file does not exist.
 */
Eigen::Matrix4f read_pose_from_file(const fs::path& filename)
{
  // Check whether or not the file exists, and throw if not.
  if(!fs::is_regular(filename))
  {
    throw std::runtime_error("Error: File not found: " + filename.string());
  }

  std::ifstream in(filename.c_str());

  // Read in the matrix (in row-major order).
  Eigen::Matrix4f res;
  in >> res(0, 0) >> res(0, 1) >> res(0, 2) >> res(0, 3);
  in >> res(1, 0) >> res(1, 1) >> res(1, 2) >> res(1, 3);
  in >> res(2, 0) >> res(2, 1) >> res(2, 2) >> res(2, 3);
  in >> res(3, 0) >> res(3, 1) >> res(3, 2) >> res(3, 3);

  // If anything went wrong during parsing, set all entries of the matrix to NaN.
  // FIXME: This is a hack, based on the fact that some of the matrices we're reading containing NaNs, and
  //        they're not saved in a consistent / platform-independent way. Trying to read them in will cause
  //        the stream to fail, so we assume that whenever the stream fails, that was the cause, and set all
  //        entries to NaN accordingly. We should fix this at some point, but it's fine for now.
  if(!in) res.setConstant(std::numeric_limits<float>::quiet_NaN());

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
float angular_separation(const Eigen::Matrix3f& r1, const Eigen::Matrix3f& r2)
{
  // First calculate the rotation matrix which maps r1 to r2.
  Eigen::Matrix3f dr = r2 * r1.transpose();

  // If the relative rotation matrix contains NaN entries, return the worst possible angular separation (i.e. PI radians).
  if(std::isnan(dr.sum())) return static_cast<float>(M_PI);

  // Otherwise, compute the corresponding angle-axis transform.
  Eigen::AngleAxisf aa(dr);

  // Then, compute the angular separation from the angle of this transform, and return it.
  return aa.angle() > static_cast<float>(M_PI) ? 2.0f * static_cast<float>(M_PI) - aa.angle() : aa.angle();
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
bool pose_matches(const Eigen::Matrix4f& gtPose, const Eigen::Matrix4f& testPose, float& translationError, float& angleError)
{
  // 7-scenes thresholds.
  static const float translationMaxError = 0.05f;
  static const float angleMaxError = 5.0f * static_cast<float>(M_PI) / 180.0f;

  const Eigen::Matrix3f gtR = gtPose.block<3, 3>(0, 0);
  const Eigen::Matrix3f testR = testPose.block<3, 3>(0, 0);
  const Eigen::Vector3f gtT = gtPose.block<3, 1>(0, 3);
  const Eigen::Vector3f testT = testPose.block<3, 1>(0, 3);

  // Compute the difference between the transformations.
  translationError = (gtT - testT).norm();
  angleError = angular_separation(gtR, testR);

  if(!std::isfinite(translationError)) translationError = std::numeric_limits<float>::infinity();
  if(!std::isfinite(angleError)) angleError = static_cast<float>(M_PI);

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
bool pose_matches(const Eigen::Matrix4f& gtPose, const Eigen::Matrix4f& testPose)
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
bool pose_file_matches(const Eigen::Matrix4f& gtPose, const fs::path& poseFile)
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
bool pose_file_matches(const Eigen::Matrix4f& gtPose, const fs::path& poseFile, float& translationError, float& angleError)
{
  translationError = std::numeric_limits<float>::infinity();
  angleError = static_cast<float>(M_PI);

  if(!fs::is_regular(poseFile)) return false;

  const Eigen::Matrix4f otherPose = read_pose_from_file(poseFile);
  return pose_matches(gtPose, otherPose, translationError, angleError);
}

/**
 * \brief Struct used to accumulate stats on a dataset sequence.
 */
struct SequenceResults
{
  //#################### PUBLIC VARIABLES ####################

  /** The average time taken during the ICP Refinement phase. */
  float averageICPRefinementTime;

  /** The average time taken during the initial relocalisation phase. */
  float averageInitialRelocalisationTime;

  /** The average time taken during the relocalisation phase. */
  float averageTotalRelocalisationTime;

  /** The average time taken during the training phase. */
  float averageTrainingTime;

  /** The average time taken during the update phase. */
  float averageUpdateTime;

  /** The number of poses in the sequence. */
  int poseCount;

  /** The number of frames successfully relocalised. */
  int validPosesAfterReloc;

  /** The number of frames successfully relocalised after a round of ICP. */
  int validPosesAfterICP;

  /** The number of frames successfully relocalised after a round of ICP+SVM. */
  int validFinalPoses;

  /** The sum of translational errors in the sequence, used to compute the average. */
  float sumRelocalisationTranslationalError;

  /** The sum of angular errors in the sequence, used to compute the average. */
  float sumRelocalisationAngleError;

  /** The sum of translational errors in the sequence, for successful relocalisations. Used to compute the average. */
  float sumRelocalisationSuccessfulTranslationalError;

  /** The sum of angular errors in the sequence, for successful relocalisations. Used to compute the average. */
  float sumRelocalisationSuccessfulAngleError;

  /** The sum of translational errors in the sequence, for failed relocalisations. Used to compute the average. */
  float sumRelocalisationFailedTranslationalError;

  /** The sum of angular errors in the sequence, for failed relocalisations. Used to compute the average. */
  float sumRelocalisationFailedAngleError;

  /** The median angle error for the sequence. */
  float medianRelocalisationAngle;

  /** The median translation error for the sequence. */
  float medianRelocalisationTranslation;

  /** The median of the finite angle error for the sequence. */
  float medianFiniteRelocalisationAngle;

  /** The median of the finite translation error for the sequence. */
  float medianFiniteRelocalisationTranslation;

  /** The median angle error for the sequence, computed after ICP. */
  float medianICPAngle;

  /** The median translation error for the sequence, computed after ICP. */
  float medianICPTranslation;

  /** The sequence of relocalisation results. Same element count as poseCount. */
  std::vector<bool> relocalizationResults;

  /** The sequence of relocalisation results, after ICP. Same element count as poseCount. */
  std::vector<bool> icpResults;

  /** The sequence of relocalisation results, after ICP+SVM. Same element count as poseCount. */
  std::vector<bool> finalResults;

  /** The sequence of angular errors after relocalisation. */
  std::vector<float> relocalisationAngularErrors;

  /** The sequence of translational errors after relocalisation. */
  std::vector<float> relocalisationTranslationalErrors;

  /** The sequence of angular errors after ICP. */
  std::vector<float> icpAngularErrors;

  /** The sequence of translational errors after ICP. */
  std::vector<float> icpTranslationalErrors;

  //#################### CONSTRUCTORS ####################

  SequenceResults()
  : averageICPRefinementTime(0),
    averageInitialRelocalisationTime(0),
    averageTotalRelocalisationTime(0),
    averageTrainingTime(0),
    averageUpdateTime(0),
    poseCount(0),
    validPosesAfterReloc(0),
    validPosesAfterICP(0),
    validFinalPoses(0),
    sumRelocalisationTranslationalError(0),
    sumRelocalisationAngleError(0),
    sumRelocalisationSuccessfulTranslationalError(0),
    sumRelocalisationSuccessfulAngleError(0),
    sumRelocalisationFailedTranslationalError(0),
    sumRelocalisationFailedAngleError(0),
    medianRelocalisationAngle(0),
    medianRelocalisationTranslation(0),
    medianFiniteRelocalisationAngle(0),
    medianFiniteRelocalisationTranslation(0),
    medianICPAngle(0),
    medianICPTranslation(0)
  {}
};

struct Comparer
{
  bool operator()(float a, float b) const
  {
    if(std::isfinite(a) && std::isfinite(b)) return a < b;

    if(std::isfinite(a)) return true;

    if(std::isfinite(b)) return false;

    return false;
  }
};

/**
 * \brief Process a dataset sequence computing how well the relocaliser performed on it.
 *
 * \param gtFolder    The folder storing the ground truth poses.
 * \param relocFolder The folder storing the relocalisation results.
 * \param statsFile   The file storing the average timings for the sequence (can be empty).
 *
 * \return The SequenceResults on this sequence.
 */
SequenceResults evaluate_sequence(const fs::path& gtFolder, const fs::path& relocFolder, const fs::path& statsFile)
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
    const fs::path finalPath = relocPathGenerator.make_path("pose-%06i.final.txt");

    // If the GT file is missing this sequence is over.
    if(!fs::is_regular(gtPath)) break;

    // Read the GT camera pose.
    const Eigen::Matrix4f gtPose = read_pose_from_file(gtPath);

    float relocalisationTranslationError, relocalisationAngleError;
    float icpTranslationError, icpAngleError;

    // Check whether different kinds of relocalisations succeeded.
    bool validReloc = pose_file_matches(gtPose, relocPath, relocalisationTranslationError, relocalisationAngleError);
    bool validICP = pose_file_matches(gtPose, icpPath, icpTranslationError, icpAngleError);
    bool validFinal = pose_file_matches(gtPose, finalPath);

    // Accumulate stats.
    res.validPosesAfterReloc += validReloc;
    res.validPosesAfterICP += validICP;
    res.validFinalPoses += validFinal;

    res.relocalizationResults.push_back(validReloc);
    res.icpResults.push_back(validICP);
    res.finalResults.push_back(validFinal);

    res.sumRelocalisationTranslationalError += relocalisationTranslationError;
    res.sumRelocalisationAngleError += relocalisationAngleError;

    if(validReloc)
    {
      res.sumRelocalisationSuccessfulTranslationalError += relocalisationTranslationError;
      res.sumRelocalisationSuccessfulAngleError += relocalisationAngleError;
    }
    else
    {
      res.sumRelocalisationFailedTranslationalError += relocalisationTranslationError;
      res.sumRelocalisationFailedAngleError += relocalisationAngleError;
    }

    res.relocalisationTranslationalErrors.push_back(relocalisationTranslationError);
    res.relocalisationAngularErrors.push_back(relocalisationAngleError);

    res.icpTranslationalErrors.push_back(icpTranslationError);
    res.icpAngularErrors.push_back(icpAngleError);

    // Increment counters.
    ++res.poseCount;
    gtPathGenerator.increment_index();
    relocPathGenerator.increment_index();
  }

  // Compute medians.
  std::vector<float> angleErrors = res.relocalisationAngularErrors;
  std::vector<float> translationErrors = res.relocalisationTranslationalErrors;

  std::vector<float> icpAngleErrors = res.icpAngularErrors;
  std::vector<float> icpTranslationErrors = res.icpTranslationalErrors;

//  std::vector<float> angleErrors;
//  std::vector<float> translationErrors;

//  for(size_t i = 0; i < res.relocalisationAngularErrors.size(); ++i)
//  {
//      float angleError = res.relocalisationAngularErrors[i];
//      float translationError = res.relocalisationTranslationalErrors[i];

//      if(std::isfinite(angleError) && std::isfinite(translationError))
//      {
//          angleErrors.push_back(angleError);
//          translationErrors.push_back(translationError);
//      }
//  }

  std::sort(angleErrors.begin(), angleErrors.end(), Comparer());
  std::sort(translationErrors.begin(), translationErrors.end(), Comparer());

  std::sort(icpAngleErrors.begin(), icpAngleErrors.end(), Comparer());
  std::sort(icpTranslationErrors.begin(), icpTranslationErrors.end(), Comparer());

  if(angleErrors.size() > 0)
  {
      size_t medianElement = angleErrors.size() / 2;
      res.medianRelocalisationAngle = angleErrors[medianElement];
      res.medianRelocalisationTranslation = translationErrors[medianElement];
      res.medianICPAngle = icpAngleErrors[medianElement];
      res.medianICPTranslation = icpTranslationErrors[medianElement];

      size_t maxValidIndex = -1;
      for(maxValidIndex = 0; maxValidIndex < angleErrors.size(); ++maxValidIndex)
      {
          if(!std::isfinite(angleErrors[maxValidIndex]) || !std::isfinite(translationErrors[maxValidIndex]))
          {
              break;
          }
      }

      if(maxValidIndex >= 0)
      {
          size_t medianElement = maxValidIndex / 2;
          res.medianFiniteRelocalisationAngle = angleErrors[medianElement];
          res.medianFiniteRelocalisationTranslation = translationErrors[medianElement];
      }
  }

  // Read stats (if available).
  if(fs::is_regular_file(statsFile))
  {
    std::ifstream in(statsFile.string().c_str());
    in >> res.averageTrainingTime >> res.averageUpdateTime >> res.averageInitialRelocalisationTime >> res.averageICPRefinementTime >> res.averageTotalRelocalisationTime;

    // Times above are in microseconds. Convert them to milliseconds.
    res.averageICPRefinementTime /= 1000.0f;
    res.averageInitialRelocalisationTime /= 1000.0f;
    res.averageTotalRelocalisationTime /= 1000.0f;
    res.averageTrainingTime /= 1000.0f;
    res.averageUpdateTime /= 1000.0f;
  }

  return res;
}

/**
 * \brief Print a variable allocating to it a certain width on screen.
 */
template <typename T>
void printWidth(const T& item, int width, bool leftAlign = false)
{
  std::cerr << (leftAlign ? std::left : std::right) << std::setw(width) << std::fixed << std::setprecision(3) << item;
}

int main(int argc, char *argv[])
{
  fs::path datasetFolder;
  fs::path relocBaseFolder;
  fs::path statsBaseFolder;
  std::string relocTag;
  bool useValidation = false;
  bool onlineEvaluation = false;
  bool verbose = false;

  // Declare some options for the evaluation.
  po::options_description options("Relocperf Options");
  options.add_options()
      ("datasetFolder,d", po::value(&datasetFolder)->required(), "The path to the dataset.")
      ("relocBaseFolder,r", po::value(&relocBaseFolder)->required(), "The path to the folder where the relocalised poses are stored.")
      ("statsBaseFolder,s", po::value(&statsBaseFolder), "The path to the folder where the relocalisation times are stored.")
      ("relocTag,t", po::value(&relocTag)->required(), "The tag assigned to the experimento to evaluate.")
      ("useValidation,v", po::bool_switch(&useValidation), "Whether to use the validation sequence to evaluate the relocaliser.")
      ("onlineEvaluation,o", po::bool_switch(&onlineEvaluation), "Whether to save the CSV for the evaluation of online relocalisation.")
      ("verbose", po::bool_switch(&verbose), "whether or not to print more informations on the sequences.")
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

  // If the tag has a .ini extension (i.e. was filled via shell autocompletion), strip it.
  if(boost::algorithm::ends_with(relocTag, ".ini"))
  {
    boost::algorithm::erase_last(relocTag, ".ini");
  }

  // Find the valid sequences in the dataset folder.
  const std::vector<std::string> sequenceNames = find_sequence_names(datasetFolder);
  int sequenceNameMaxLength = 0;

  std::map<std::string, SequenceResults> results;

  // Evaluate each sequence.
  for(size_t sequenceIdx = 0; sequenceIdx < sequenceNames.size(); ++sequenceIdx)
  {
    const std::string& sequence = sequenceNames[sequenceIdx];

    sequenceNameMaxLength = std::max(sequenceNameMaxLength, static_cast<int>(sequence.length()) + 2);

    // Compute the full paths.
    const fs::path gtPath = datasetFolder / sequence / (useValidation ? validationFolderName : testFolderName);
    const fs::path relocFolder = relocBaseFolder / (relocTag + '_' + sequence);
    const fs::path statsFile = statsBaseFolder / (relocTag + '_' + sequence + ".txt");

    std::cerr << "Processing sequence " << sequence << " in: " << gtPath << "\t - " << relocFolder << std::endl;
    try
    {
      results[sequence] = evaluate_sequence(gtPath, relocFolder, statsFile);
    }
    catch(std::runtime_error &)
    {
      std::cerr << "\tSequence has not been evaluated.\n";
    }
  }

  // Print table
  printWidth("Sequence", sequenceNameMaxLength, true);
  printWidth("Poses", 8);
  printWidth("Reloc", 8);
  printWidth("ICP", 8);

  if(verbose)
  {
    printWidth("Final", 8);
    printWidth("Avg_T.", 20);
    printWidth("Avg_A.", 8);
    printWidth("Avg_Succ_T.", 14);
    printWidth("Avg_Succ_A.", 14);
    printWidth("Avg_Fail_T.", 14);
    printWidth("Avg_Fail_A.", 14);
  }

  printWidth("Median_Reloc_T.", 17);
  printWidth("Median_Reloc_A.", 17);
  printWidth("Median_ICP_T.", 17);
  printWidth("Median_ICP_A.", 17);

  printWidth("Training_ms", 20);
  printWidth("Update_ms", 20);
  printWidth("Initial_Reloc_ms", 20);
  printWidth("ICP_ms", 20);
  printWidth("Total_Reloc_ms", 20);

  std::cerr << '\n';

  // Compute percentages for each sequence and print everything.
  for(const auto& sequence : sequenceNames)
  {
    const auto& seqResult = results[sequence];

    float relocPct = static_cast<float>(seqResult.validPosesAfterReloc) / static_cast<float>(seqResult.poseCount) * 100.f;
    float icpPct = static_cast<float>(seqResult.validPosesAfterICP) / static_cast<float>(seqResult.poseCount) * 100.f;
    float finalPct = static_cast<float>(seqResult.validFinalPoses) / static_cast<float>(seqResult.poseCount) * 100.f;

    float avgTranslation = seqResult.sumRelocalisationTranslationalError / static_cast<float>(seqResult.poseCount);
    float avgAngle = (seqResult.sumRelocalisationAngleError / static_cast<float>(seqResult.poseCount)) * 180 / static_cast<float>(M_PI);

    float avgSuccTranslation = seqResult.sumRelocalisationSuccessfulTranslationalError / static_cast<float>(seqResult.validPosesAfterReloc);
    float avgSuccAngle = (seqResult.sumRelocalisationSuccessfulAngleError / static_cast<float>(seqResult.validPosesAfterReloc)) * 180 / static_cast<float>(M_PI);

    float avgFailTranslation = seqResult.sumRelocalisationFailedTranslationalError / static_cast<float>(seqResult.poseCount - seqResult.validPosesAfterReloc);
    float avgFailAngle = (seqResult.sumRelocalisationFailedAngleError / static_cast<float>(seqResult.poseCount - seqResult.validPosesAfterReloc)) * 180 / static_cast<float>(M_PI);

    float medianAngleError = seqResult.medianRelocalisationAngle * 180 / static_cast<float>(M_PI);
    float medianTranslationError = seqResult.medianRelocalisationTranslation;

    float medianICPAngleError = seqResult.medianICPAngle * 180 / static_cast<float>(M_PI);
    float medianICPTranslationError = seqResult.medianICPTranslation;

//    float medianFiniteAngleError = seqResult.medianFiniteRelocalisationAngle * 180 / M_PI;
//    float medianFiniteTranslationError = seqResult.medianFiniteRelocalisationTranslation;

    printWidth(sequence, sequenceNameMaxLength, true);
    printWidth(seqResult.poseCount, 8);
    printWidth(relocPct, 8);
    printWidth(icpPct, 8);

    if(verbose)
    {
      printWidth(finalPct, 8);
      printWidth(avgTranslation, 20);
      printWidth(avgAngle, 8);
      printWidth(avgSuccTranslation, 14);
      printWidth(avgSuccAngle, 14);
      printWidth(avgFailTranslation, 14);
      printWidth(avgFailAngle, 14);
    }

    printWidth(medianTranslationError, 17);
    printWidth(medianAngleError, 17);
    printWidth(medianICPTranslationError, 17);
    printWidth(medianICPAngleError, 17);

    printWidth(seqResult.averageTrainingTime, 20);
    printWidth(seqResult.averageUpdateTime, 20);
    printWidth(seqResult.averageInitialRelocalisationTime, 20);
    printWidth(seqResult.averageICPRefinementTime, 20);
    printWidth(seqResult.averageTotalRelocalisationTime, 20);

    std::cerr << '\n';
  }

  // Compute average performance.
  float relocSum = 0.f;
  float icpSum = 0.f;
  float finalSum = 0.f;

  float relocRawSum = 0.f;
  float icpRawSum = 0.f;
  float finalRawSum = 0.f;

  float medianAngleSum = 0.f;
  float medianTranslationSum = 0.f;
  float medianICPAngleSum = 0.f;
  float medianICPTranslationSum = 0.f;

  float averageICPTimeSum = 0.0f;
  float averageInitialRelocTimeSum = 0.0f;
  float averageTrainingTimeSum = 0.0f;
  float averageTotalRelocTimeSum = 0.0f;
  float averageUpdateTimeSum = 0.0f;

  float relocLoss = 0.0f;
  float icpLoss = 0.0f;

  int poseCount = 0;

  for(const auto& sequence : sequenceNames)
  {
    const auto& seqResult = results[sequence];

    // Non-weighted average, we need percentages
    const float relocPct = static_cast<float>(seqResult.validPosesAfterReloc) / static_cast<float>(seqResult.poseCount);
    const float icpPct = static_cast<float>(seqResult.validPosesAfterICP) / static_cast<float>(seqResult.poseCount);
    const float finalPct = static_cast<float>(seqResult.validFinalPoses) / static_cast<float>(seqResult.poseCount);

    medianAngleSum += seqResult.medianRelocalisationAngle;
    medianTranslationSum += seqResult.medianRelocalisationTranslation;
    medianICPAngleSum += seqResult.medianICPAngle;
    medianICPTranslationSum += seqResult.medianICPTranslation;

    relocSum += relocPct;
    icpSum += icpPct;
    finalSum += finalPct;

    relocRawSum += static_cast<float>(seqResult.validPosesAfterReloc);
    icpRawSum += static_cast<float>(seqResult.validPosesAfterICP);
    finalRawSum += static_cast<float>(seqResult.validFinalPoses);

    relocLoss += std::isfinite(relocPct) ? std::pow(1.0f - relocPct, 2) : 1.0f;
    icpLoss += std::isfinite(icpPct) ? std::pow(1.0f - icpPct, 2) : 1.0f;

    poseCount += seqResult.poseCount;

    averageICPTimeSum += seqResult.averageICPRefinementTime;
    averageInitialRelocTimeSum += seqResult.averageInitialRelocalisationTime;
    averageTotalRelocTimeSum += seqResult.averageTotalRelocalisationTime;
    averageTrainingTimeSum += seqResult.averageTrainingTime;
    averageUpdateTimeSum += seqResult.averageUpdateTime;
  }

  const float relocAvg = relocSum / sequenceNames.size() * 100.0f;
  const float icpAvg = icpSum / sequenceNames.size() * 100.0f;
  const float finalAvg = finalSum / sequenceNames.size() * 100.0f;

  const float relocWeightedAvg = relocRawSum / poseCount * 100.0f;
  const float icpWeightedAvg = icpRawSum / poseCount * 100.0f;
  const float finalWeightedAvg = finalRawSum / poseCount * 100.0f;

  const float medianAngleAvg = medianAngleSum / sequenceNames.size() * 180.0f / static_cast<float>(M_PI);
  const float medianTranslationAvg = medianTranslationSum / sequenceNames.size();
  const float medianICPAngleAvg = medianICPAngleSum / sequenceNames.size() * 180.0f / static_cast<float>(M_PI);
  const float medianICPTranslationAvg = medianICPTranslationSum / sequenceNames.size();

  const float averageICPTime = averageICPTimeSum / sequenceNames.size();
  const float averageInitialRelocTime = averageInitialRelocTimeSum / sequenceNames.size();
  const float averageTrainingTime = averageTrainingTimeSum / sequenceNames.size();
  const float averageTotalRelocTime = averageTotalRelocTimeSum / sequenceNames.size();
  const float averageUpdateTime = averageUpdateTimeSum / sequenceNames.size();

  // Print averages
  std::cerr << '\n';
  printWidth("Average", sequenceNameMaxLength, true);
  printWidth(sequenceNames.size(), 8);
  printWidth(relocAvg, 8);
  printWidth(icpAvg, 8);

  if(verbose)
  {
    printWidth(finalAvg, 8);
    printWidth(std::numeric_limits<float>::quiet_NaN(), 20);
    printWidth(std::numeric_limits<float>::quiet_NaN(), 8);
    printWidth(std::numeric_limits<float>::quiet_NaN(), 14);
    printWidth(std::numeric_limits<float>::quiet_NaN(), 14);
    printWidth(std::numeric_limits<float>::quiet_NaN(), 14);
    printWidth(std::numeric_limits<float>::quiet_NaN(), 14);
  }

  printWidth(medianTranslationAvg, 17);
  printWidth(medianAngleAvg, 17);
  printWidth(medianICPTranslationAvg, 17);
  printWidth(medianICPAngleAvg, 17);

  printWidth(averageTrainingTime, 20);
  printWidth(averageUpdateTime, 20);
  printWidth(averageInitialRelocTime, 20);
  printWidth(averageICPTime, 20);
  printWidth(averageTotalRelocTime, 20);

  std::cerr << '\n';

  printWidth("Average_(W)", sequenceNameMaxLength, true);
  printWidth(poseCount, 8);
  printWidth(relocWeightedAvg, 8);
  printWidth(icpWeightedAvg, 8);

  if(verbose)
  {
    printWidth(finalWeightedAvg, 8);
  }

  std::cerr << '\n';

  // Print the weighted averages for the parameter search algorithm.
  if(useValidation)
  {
    std::cout.unsetf(std::ios_base::floatfield);
    std::cout << relocLoss << ' ' << icpLoss << '\n';
  }

  // Save results of online training-relocalization
  if(onlineEvaluation)
  {
    std::string onlineResultsFilenameStem = relocTag;

    // Process every sequence
    for(auto sequence : sequenceNames)
    {
      auto seqResult = results[sequence];

      std::string outFilename = onlineResultsFilenameStem + '_' + sequence + ".csv";
      std::ofstream out(outFilename.c_str());

      // Print header
      out << "FrameIdx, FramePct, Reloc Success, Reloc Translation, Reloc Angle, Reloc Sum, Reloc Pct, ICP Success, "
             "ICP Sum, ICP Pct\n";

      int relocSum = 0;
      int icpSum = 0;

      for(int poseIdx = 0; poseIdx < seqResult.poseCount; ++poseIdx)
      {
        bool relocSuccess = seqResult.relocalizationResults[poseIdx];
        bool icpSuccess = seqResult.icpResults[poseIdx];

        float relocTranslation = seqResult.relocalisationTranslationalErrors[poseIdx];
        float relocAngle = seqResult.relocalisationAngularErrors[poseIdx] * 180 / static_cast<float>(M_PI);

        relocSum += relocSuccess;
        icpSum += icpSuccess;

        float framePct = static_cast<float>(poseIdx) / seqResult.poseCount;
        float relocPct = static_cast<float>(relocSum) / poseIdx;
        float icpPct = static_cast<float>(icpSum) / poseIdx;

        out << poseIdx << ", " << framePct << ", " << relocSuccess << ", " << relocTranslation << ", " << relocAngle
            << ", " << relocSum << ", " << relocPct << ", " << icpSuccess << ", " << icpSum << ", " << icpPct << '\n';
      }
    }
  }

#if 0
  std::cout << "\n\n";

  float colorMin = 40;
  float colorMax = 95;
  std::string relocColor = "ForestGreen";
  std::string icpColor = "CornflowerBlue";

  // Print Latex table row
  std::cout << "\\cellcolor{white} & \\cellcolor{" << relocColor << "} Reloc ";
  for (auto sequence : sequenceNames)
  {
    auto seqResult = results[sequence];

    float relocPct = static_cast<float>(seqResult.validPosesAfterReloc)
    / static_cast<float>(seqResult.poseCount) * 100.f;
    float relocColorPct = (relocPct / 100.f) * (colorMax - colorMin) + colorMin;

    std::cout << "& \\cellcolor{" << relocColor << "!" << relocColorPct << "} "
    << std::setprecision(1) << relocPct << "\\% ";
  }

  std::cout << "\\\\\n\\cellcolor{white}\\multirow{-2}{*}{" << relocTag
  << "} & \\cellcolor{" << icpColor << "} + ICP ";
  for (auto sequence : sequenceNames)
  {
    auto seqResult = results[sequence];

    float icpPct = static_cast<float>(seqResult.validPosesAfterICP)
    / static_cast<float>(seqResult.poseCount) * 100.f;
    float icpColorPct = (icpPct / 100.f) * (colorMax - colorMin) + colorMin;

    std::cout << "& \\cellcolor{" << icpColor << "!" << icpColorPct << "} "
    << std::setprecision(1) << icpPct << "\\% ";
  }

  std::cout << "\\\\\n";
#endif

  return 0;
}
