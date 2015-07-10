/**
 * raflperf: main.cpp
 */

#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

#include <tvgutil/SerializationUtil.h>

#include <evaluation/core/PerformanceTable.h>
#include <evaluation/splitgenerators/CrossValidationSplitGenerator.h>
#include <evaluation/splitgenerators/RandomPermutationAndDivisionSplitGenerator.h>
#include <evaluation/util/CartesianProductParameterSetGenerator.h>
using namespace evaluation;

#include <rafl/examples/ExampleUtil.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

#include <raflevaluation/RandomForestEvaluator.h>
using namespace raflevaluation;

#include <tvgutil/timers/Timer.h>
#include <tvgutil/TimeUtil.h>

#include "TouchTrainUtil.h"

//#################### TYPEDEFS ####################

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
typedef std::vector<std::pair<std::string, Label> > Instances;
typedef CartesianProductParameterSetGenerator::ParamSet ParamSet;

//#################### FUNCTIONS ####################

/**
 * \brief Checks whether the specified path exists.
 *
 * If the path is not found, it prints the expected path to std::cout.
 *
 * \param path  The path.
 * \return      True, if the path exists, false otherwise.
 */
bool check_path_exists(const std::string& path)
{
  if(!boost::filesystem::exists(path))
  {
    std::cout << "[touchtrain] Expecting to see: " << path << std::endl;
    return false;
  }
  else
  {
    return true;
  }
}

/**
 * \brief A struct that represents the file structure for the touch training data.
 */
struct TouchTrainData
{
  //#################### TYPEDEFS ####################

  typedef std::vector<LabelledImagePath<Label> > InstanceSet;

  //#################### PUBLIC VARIABLES ####################

  /** The directory containing tables of results generated during cross-validation. */
  std::string m_crossValidationResults;

  /** An array of instances, composed of <path-to-image,label>. */
  std::vector<InstanceSet> m_instanceSets;

  /** The directory where the forest models are stored. */
  std::string m_models;

  /** The root directory in which the touch training data is stored. */
  std::string m_root;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs the paths and data relevant for touch training.
   *
   * \param root             The root directory containing the touch training data.
   * \param sequenceNumbers  An array containing the sequence numbers to be included during training.
   */
  TouchTrainData(const std::string& root, std::vector<size_t> sequenceNumbers)
  : m_instanceSets(sequenceNumbers.size()), m_root(root)
  {
    size_t invalidCount = 0;

    m_crossValidationResults = root + "/crossvalidation-results";
    if(!check_path_exists(m_crossValidationResults)) ++invalidCount;

    m_models = root + "/models";
    if(!check_path_exists(m_models)) ++invalidCount;

    boost::format threeDigits("%03d");
    for(size_t i = 0, size = sequenceNumbers.size(); i < size; ++i)
    {
      std::string sequencePath = root + "/seq" + (threeDigits % sequenceNumbers[i]).str();
      if(!check_path_exists(sequencePath)) ++invalidCount;

      std::string imagePath = sequencePath + "/images";
      std::string annotationPath = sequencePath + "/annotation.txt";
      if(!check_path_exists(imagePath)) ++invalidCount;
      if(!check_path_exists(annotationPath)) ++invalidCount;

      m_instanceSets[i] = TouchTrainUtil::load_instances<Label>(imagePath, annotationPath);
      if(m_instanceSets[i].empty())
      {
        std::cout << "[touchtrain] Expecting some data in: " << sequencePath << std::endl;
        ++invalidCount;
      }
    }

    if(invalidCount > 0)
    {
      throw std::runtime_error("The aforementioned directories were not found, please create and populate them.");
    }
  }
};

int main(int argc, char *argv[])
{
#if WITH_OPENMP
  omp_set_nested(1);
#endif

  const unsigned int seed = 12345;

  if(argc != 2)
  {
    std::cerr << "Usage: raflperf [<touch training set path>]\n";
    return EXIT_FAILURE;
  }

  const size_t treeCount = 8;
  const size_t splitBudget = 1048576/2;

  TouchTrainData touchDataset(argv[1],list_of(2)(3));
  std::cout << "[touchtrain] Training set root: " << touchDataset.m_root << '\n';

  std::cout << "[touchtrain] Generating examples...\n";
  std::vector<Example_CPtr> examples = TouchTrainUtil::generate_examples<Label>(touchDataset.m_instanceSets);
  std::cout << "[touchtrain] Number of examples = " << examples.size() << '\n';

  // Generate the parameter sets with which to test the random forest.
  std::vector<ParamSet> params = CartesianProductParameterSetGenerator()
    .add_param("treeCount", list_of<size_t>(treeCount))
    .add_param("splitBudget", list_of<size_t>(splitBudget))
    .add_param("candidateCount", list_of<int>(256))
    .add_param("decisionFunctionGeneratorParams", list_of<std::string>(""))
    .add_param("decisionFunctionGeneratorType", list_of<std::string>("FeatureThresholding"))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(1000))
    .add_param("maxTreeHeight", list_of<size_t>(20))
    .add_param("randomSeed", list_of<unsigned int>(seed))
    .add_param("seenExamplesThreshold", list_of<size_t>(32)(64)(128))
    .add_param("splittabilityThreshold", list_of<float>(0.3f)(0.5f)(0.8f))
    .add_param("usePMFReweighting", list_of<bool>(false)(true))
    .generate_param_sets();

  // Register the relevant decision function generators with the factory.
  DecisionFunctionGeneratorFactory<Label>::instance().register_rafl_makers();

  // Construct the split generator.
#if 1
  const size_t foldCount = 5;
  SplitGenerator_Ptr splitGenerator(new CrossValidationSplitGenerator(seed, foldCount));
#else
  const size_t splitCount = 5;
  const float ratio = 0.5f;
  SplitGenerator_Ptr splitGenerator(new RandomPermutationAndDivisionSplitGenerator(seed, splitCount, ratio));
#endif

  // Time the random forest.
  tvgutil::Timer<boost::chrono::seconds> timer("ForestEvaluationTime");

  // Evaluate the random forest on the various different parameter sets.
  std::cout << "[touchtrain] Cross-validating the performance of the forest on various parameter sets...\n";
  PerformanceTable results(list_of("Accuracy"));
  boost::shared_ptr<RandomForestEvaluator<Label> > evaluator;
  for(size_t n = 0, size = params.size(); n < size; ++n)
  {
    evaluator.reset(new RandomForestEvaluator<Label>(splitGenerator, params[n]));
    std::map<std::string,PerformanceMeasure> result = evaluator->evaluate(examples);
    results.record_performance(params[n], result);
  }

  // Output the performance table to the screen.
  results.output(std::cout); std::cout << '\n';

  timer.stop();
  std::cout << "[touchtrain] " << timer << '\n';

  // Get a time-stamp for tagging the resulting files.
  const std::string timeStamp = tvgutil::TimeUtil::get_iso_timestamp();

  // Time-stamp the results file.
  std::string textOutputResultPath =  touchDataset.m_crossValidationResults + "/crossvalidationresults-" + timeStamp + ".txt";

  // Output the performance table to the results file.
  std::ofstream resultsFile(textOutputResultPath.c_str());
  if(!resultsFile)
  {
    std::cout << "[touchtrain] Warning could not open file for writing...\n";
  }
  else
  {
    results.output(resultsFile);
  }

  // Now Train a classifier with the best settings selected during crossvalidation.
  typedef DecisionTree<Label> DT;
  typedef RandomForest<Label> RF;
  typedef boost::shared_ptr<RF> RF_Ptr;

  std::cout << "[touchtrain] Training the forest with the best parameters selected during cross-validation...\n";
  ParamSet bestParams = results.find_best_param_set("Accuracy");
  DT::Settings settings(bestParams);
  RF_Ptr randomForest(new RF(treeCount, settings));
  randomForest->add_examples(examples);

  std::cout << "[touchtrain] The final trained forest statistics:\n";
  if(randomForest->train(splitBudget)) randomForest->output_statistics(std::cout);
  std::string forestPath = touchDataset.m_models + "/randomForest-" + timeStamp + ".rf";
  std::cout << "[touchtrain] Saving the forest to: " << forestPath << "\n";
  tvgutil::SerializationUtil::save_text(touchDataset.m_models + "/randomForest-" + timeStamp + ".rf", *randomForest);

  return 0;
}
