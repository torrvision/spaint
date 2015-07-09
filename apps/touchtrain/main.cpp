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
#include <rafl/helpers/RandomForestEvaluator.h>
using namespace rafl;

#include <tvgutil/timers/Timer.h>
#include <tvgutil/TimeUtil.h>

#include "TouchTrainUtil.h"

//#################### TYPEDEFS ####################

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
typedef std::vector<std::pair<std::string, Label> > Instances;
typedef CartesianProductParameterSetGenerator::ParamSet ParamSet;

//#################### FUNCTIONS ####################

#if 0
/**
 * \brief Gets the current time in ISO format.
 *
 * \return  The current time in ISO format.
 */
std::string get_iso_timestamp()
{
  boost::posix_time::ptime currentDateTime(boost::posix_time::second_clock::local_time());
  return boost::posix_time::to_iso_string(currentDateTime);
}
#endif

bool check_path_exists(const std::string& path)
{
  if(!boost::filesystem::exists(path))
  {
    std::cout << "Expecting to see: " << path << std::endl;
    return false;
  }
  else
  {
    return true;
  }
}

struct TouchTrainData
{
  typedef std::vector<std::pair<std::string,Label> > Instances;

  std::string m_crossValidationResults;
  std::vector<Instances> m_instances;
  std::string m_models;
  std::string m_root;

  TouchTrainData(const std::string& root, std::vector<size_t> sequenceNumbers)
  : m_instances(sequenceNumbers.size()), m_root(root) 
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

      m_instances[i] = TouchTrainUtil::load_instances<Label>(imagePath, annotationPath);
      if(m_instances[i].empty())
      {
        std::cout << "Expecting some data in: " << sequencePath << std::endl;
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

  std::vector<Example_CPtr> examples;
  std::vector<ParamSet> params;

  const size_t treeCount = 8;
  const size_t splitBudget = 1048576/2;

  TouchTrainData touchDataset(argv[1],list_of(2)(3));

  //std::string trainingSetFileName = argv[2];
  //outputResultPath = argv[3];

  std::cout << "Training set: " << touchDataset.m_root << '\n';

  examples = TouchTrainUtil::generate_examples<Label>(touchDataset.m_instances);

  std::cout << "Number of examples = " << examples.size() << '\n';

  // Generate the parameter sets with which to test the random forest.
  params = CartesianProductParameterSetGenerator()
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
  tvgutil::Timer<boost::chrono::milliseconds> timer("ForestEvaluation");

  // Evaluate the random forest on the various different parameter sets.
  PerformanceTable results(list_of("Accuracy"));
  boost::shared_ptr<RandomForestEvaluator<Label> > evaluator;
  for(size_t n = 0, size = params.size(); n < size; ++n)
  {
    evaluator.reset(new RandomForestEvaluator<Label>(splitGenerator, params[n]));
    std::map<std::string,PerformanceMeasure> result = evaluator->evaluate(examples);
    results.record_performance(params[n], result);
  }

  timer.stop();
  std::cout << timer << '\n';

  // Output the performance table to the screen.
  results.output(std::cout);

  // Get a time-stamp for tagging the resulting files.
  const std::string timeStamp = TimeUtil::get_iso_timestamp();

  // Time-stamp the results file.
  std::string textOutputResultPath =  touchDataset.m_crossValidationResults + "/crossvalidationresults-" + timeStamp + ".txt";

  // Output the performance table to the results file.
  std::ofstream resultsFile(textOutputResultPath.c_str());
  if(!resultsFile)
  {
    std::cout << "Warning could not open file for writing...\n";
  }
  else
  {
    results.output(resultsFile);
  }

  // Now Train a classifier with the best settings selected during crossvalidation.
  typedef DecisionTree<Label> DT;
  typedef RandomForest<Label> RF;
  typedef boost::shared_ptr<RF> RF_Ptr;

  ParamSet bestParams = results.find_best_params("Accuracy");
  DT::Settings settings(bestParams);
  RF_Ptr randomForest(new RF(treeCount, settings));
  randomForest->add_examples(examples);
  if(randomForest->train(splitBudget)) randomForest->output_statistics(std::cout);
  tvgutil::SerializationUtil::save_text(touchDataset.m_models + "/randomForest-" + timeStamp + ".rf", *randomForest);

  return 0;
}
