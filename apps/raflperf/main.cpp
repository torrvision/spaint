/**
 * raflperf: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <boost/assign/list_of.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

#include <evaluation/core/PerformanceTable.h>
#include <evaluation/splitgenerators/CrossValidationSplitGenerator.h>
#include <evaluation/splitgenerators/RandomPermutationAndDivisionSplitGenerator.h>
#include <evaluation/util/CartesianProductParameterSetGenerator.h>
using namespace evaluation;

#include <rafl/examples/ExampleUtil.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

#include <tvgutil/timing/Timer.h>

#include "RandomForestEvaluator.h"

//#################### TYPEDEFS ####################

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
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

  if(argc != 1 && argc != 4)
  {
    std::cerr << "Usage: raflperf [<training set file> <test set file> <output path>]\n";
    return EXIT_FAILURE;
  }

  std::vector<Example_CPtr> examples;
  std::vector<ParamSet> params;
  std::string outputResultPath;

  if(argc == 1)
  {
#define CLASS_IMBALANCE_TEST

    // Generate a set of labels.
    std::set<Label> classLabels;
    classLabels.insert(1);
    classLabels.insert(3);
    classLabels.insert(5);
    classLabels.insert(7);

    // Generate examples around the unit circle.
    UnitCircleExampleGenerator<Label> uceg(classLabels, seed);

#ifdef CLASS_IMBALANCE_TEST
    const int biasedClass = 7;
    std::set<Label> biasedClassLabels;
    biasedClassLabels.insert(biasedClass);

    std::set<Label> unbiasedClassLabels = classLabels;
    unbiasedClassLabels.erase(biasedClass);

    examples = uceg.generate_examples(unbiasedClassLabels, 1000);
    std::vector<Example_CPtr> biasedExamples = uceg.generate_examples(biasedClassLabels, 100000);
    examples.insert(examples.end(), biasedExamples.begin(), biasedExamples.end());
#else
    examples = uceg.generate_examples(classLabels, 1000);
#endif

    params = CartesianProductParameterSetGenerator()
      .add_param("treeCount", list_of<size_t>(2))
      .add_param("splitBudget", list_of<size_t>(1048576/2))
      .add_param("candidateCount", list_of<int>(256))
      .add_param("decisionFunctionGeneratorParams", list_of<std::string>(""))
      .add_param("decisionFunctionGeneratorType", list_of<std::string>("FeatureThresholding"))
      .add_param("gainThreshold", list_of<float>(0.0f))
      .add_param("maxClassSize", list_of<size_t>(1000))
      .add_param("maxTreeHeight", list_of<size_t>(20))
      .add_param("randomSeed", list_of<unsigned int>(seed))
      .add_param("seenExamplesThreshold", list_of<size_t>(50))
      .add_param("splittabilityThreshold", list_of<float>(0.8f))
      .add_param("usePMFReweighting", list_of<bool>(false)(true))
      .generate_param_sets();

    outputResultPath = "UnitCircleExampleGenerator-Results.txt";
  }
  else if(argc == 4)
  {
    std::string trainingSetPath = argv[1];
    std::string testingSetPath = argv[2];
    outputResultPath = argv[3];

    std::cout << "Training set: " << trainingSetPath << '\n';
    std::cout << "Testing set: " << testingSetPath << '\n';

    examples = ExampleUtil::load_examples<Label>(trainingSetPath);
    std::vector<Example_CPtr> testingExamples = ExampleUtil::load_examples<Label>(testingSetPath);

    examples.insert(examples.end(), testingExamples.begin(), testingExamples.end());
    std::cout << "Number of examples = " << examples.size() << '\n';

    // Generate the parameter sets with which to test the random forest.
    params = CartesianProductParameterSetGenerator()
      .add_param("treeCount", list_of<size_t>(8))
      .add_param("splitBudget", list_of<size_t>(1048576/2))
      .add_param("candidateCount", list_of<int>(256))
      .add_param("decisionFunctionGeneratorParams", list_of<std::string>(""))
      .add_param("decisionFunctionGeneratorType", list_of<std::string>("FeatureThresholding"))
      .add_param("gainThreshold", list_of<float>(0.0f))
      .add_param("maxClassSize", list_of<size_t>(100000))
      .add_param("maxTreeHeight", list_of<size_t>(1000000))
      .add_param("randomSeed", list_of<unsigned int>(seed))
      .add_param("seenExamplesThreshold", list_of<size_t>(512))
      .add_param("splittabilityThreshold", list_of<float>(0.8f))
      .add_param("usePMFReweighting", list_of<bool>(false)(true))
      .generate_param_sets();
  }

  // Register the relevant decision function generators with the factory.
  DecisionFunctionGeneratorFactory<Label>::instance().register_rafl_makers();

  // Construct the split generator.
#if 0
  const size_t foldCount = 2;
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
  outputResultPath += "-" + get_iso_timestamp();

  // Output the performance table to the results file.
  std::ofstream resultsFile(outputResultPath.c_str());
  if(!resultsFile)
  {
    std::cout << "Warning could not open file for writing...\n";
  }
  else
  {
    results.output(resultsFile);
  }

  return 0;
}
