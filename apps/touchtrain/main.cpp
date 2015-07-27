/**
 * touchtrain: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <boost/assign/list_of.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
using boost::assign::list_of;

#include <evaluation/core/PerformanceTable.h>
#include <evaluation/splitgenerators/CrossValidationSplitGenerator.h>
#include <evaluation/splitgenerators/RandomPermutationAndDivisionSplitGenerator.h>
#include <evaluation/util/CartesianProductParameterSetGenerator.h>
using namespace evaluation;

#include <rafl/decisionfunctions/DecisionFunctionGeneratorFactory.h>
using namespace rafl;

#include <raflevaluation/RandomForestEvaluator.h>
using namespace raflevaluation;

#include <spaint/touch/TouchDescriptorCalculator.h>
using namespace spaint;

#include <tvgutil/SerializationUtil.h>
#include <tvgutil/timing/Timer.h>
#include <tvgutil/timing/TimeUtil.h>
using namespace tvgutil;

#include "LabelledPath.h"
#include "TouchTrainDataset.h"

//#################### TYPEDEFS ####################

typedef int Label;
typedef DecisionTree<Label> DT;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
typedef CartesianProductParameterSetGenerator::ParamSet ParamSet;
typedef RandomForest<Label> RF;
typedef boost::shared_ptr<RF> RF_Ptr;

//#################### FUNCTIONS ####################

/**
 * \brief Generates an array of examples given an array of labelled image paths.
 *
 * \param labelledImagePaths  The array of labelled image paths.
 * \return                    The examples.
 */
std::vector<boost::shared_ptr<const Example<Label> > > generate_examples(const std::vector<LabelledPath<Label> >& labelledImagePaths)
{
  const size_t labelledImagePathCount = labelledImagePaths.size();

  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  std::vector<Example_CPtr> examples(labelledImagePathCount);

  for(size_t i = 0; i < labelledImagePathCount; ++i)
  {
    af::array img = af::loadImage(labelledImagePaths[i].path.c_str());
    Descriptor_CPtr descriptor = TouchDescriptorCalculator::calculate_histogram_descriptor(img);
    examples[i].reset(new Example<Label>(descriptor, labelledImagePaths[i].label));
  }

  return examples;
}

int main(int argc, char *argv[])
{
#if WITH_OPENMP
  omp_set_nested(1);
#endif

  const unsigned int seed = 12345;

  if(argc != 2)
  {
    std::cerr << "Usage: touchtrain [<touch training set path>]\n";
    return EXIT_FAILURE;
  }

  const size_t treeCount = 8;
  const size_t splitBudget = 1048576 / 2;

  TouchTrainDataset<Label> touchDataset(argv[1], list_of(2)(3)(4)(5));
  std::cout << "[touchtrain] Training set root: " << touchDataset.get_root_directory() << '\n';

  std::cout << "[touchtrain] Generating examples...\n";
  std::vector<Example_CPtr> examples = generate_examples(touchDataset.get_labelled_image_paths());
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
  Timer<boost::chrono::seconds> timer("ForestEvaluationTime");

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
  const std::string timeStamp = TimeUtil::get_iso_timestamp();

  // Time-stamp the results file.
  std::string textOutputResultPath =  touchDataset.get_cross_validation_results_directory() + "/crossvalidationresults-" + timeStamp + ".txt";

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

  std::cout << "[touchtrain] Training the forest with the best parameters selected during cross-validation...\n";
  ParamSet bestParams = results.find_best_param_set("Accuracy");
  DT::Settings settings(bestParams);
  RF_Ptr randomForest(new RF(treeCount, settings));
  randomForest->add_examples(examples);

  std::cout << "[touchtrain] The final trained forest statistics:\n";
  if(randomForest->train(splitBudget)) randomForest->output_statistics(std::cout);

  std::string forestPath = touchDataset.get_models_directory() + "/randomForest-" + timeStamp + ".rf";
  std::cout << "[touchtrain] Saving the forest to: " << forestPath << "\n";
  SerializationUtil::save_text(forestPath, *randomForest);

  return 0;
}
