/**
 * raflperf: main.cpp
 */

#include <boost/assign/list_of.hpp>
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

#include <tvgutil/timers/Timer.h>

#include "RandomForestEvaluator.h"
#include "TouchTrainUtil.h"

//#################### TYPEDEFS ####################

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
typedef std::vector<std::pair<std::string, Label> > Instances;
typedef CartesianProductParameterSetGenerator::ParamSet ParamSet;

//#################### FUNCTIONS ####################

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

int main(int argc, char *argv[])
{
#if WITH_OPENMP
  omp_set_nested(1);
#endif

  const unsigned int seed = 12345;

  if(argc != 4)
  {
    std::cerr << "Usage: raflperf [<touch training set path> <training set filename> <output path>]\n";
    return EXIT_FAILURE;
  }

  std::vector<Example_CPtr> examples;
  std::vector<ParamSet> params;
  std::string outputResultPath;

  const size_t treeCount = 8;
  const size_t splitBudget = 1048576/2;

  if(argc == 4)
  {
    std::string trainingSetPath = argv[1];
    std::string trainingSetFileName = argv[2];
    outputResultPath = argv[3];

    std::cout << "Training set: " << trainingSetPath << '\n';

    Instances instances = TouchTrainUtil::load_instances<Label>(trainingSetPath, trainingSetFileName);
    examples = TouchTrainUtil::generate_examples<Label>(instances);

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
  }

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

  // Time-stamp the results file.
  std::string textOutputResultPath =  outputResultPath + "/crossvalidationresults-" + get_iso_timestamp() + ".txt";

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
  const std::string rfPath = outputResultPath + "/randomForest-" + get_iso_timestamp() + ".rf"; 
  std::cout << rfPath << std::endl;
  tvgutil::SerializationUtil::save_text(rfPath, *randomForest);

  return 0;
}
