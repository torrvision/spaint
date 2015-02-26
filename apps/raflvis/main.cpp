/**
 * raflvis: main.cpp
 */

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <boost/format.hpp>

#include <evaluation/util/ConfusionMatrixUtil.h>
#include <evaluation/util/CartesianProductParameterSetGenerator.h>
#include <evaluation/core/PerformanceMeasure.h>
using namespace evaluation;

#include <rafl/examples/ExampleUtil.h>
#include <rafl/RandomForest.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using rafl::Descriptor;
using rafl::Descriptor_Ptr;
using rafl::Descriptor_CPtr;

#include <tvgutil/RandomNumberGenerator.h>

#include "PlotWindow.h"
#include "PaletteGenerator.h"

//#################### TYPEDEFS ####################
typedef int Label;
typedef boost::shared_ptr<const rafl::Example<Label> > Example_CPtr;
typedef evaluation::CartesianProductParameterSetGenerator::ParamSet ParamSet;
typedef rafl::DecisionTree<Label> DecisionTree;
typedef rafl::RandomForest<Label> RandomForest;
typedef boost::shared_ptr<RandomForest> RandomForest_Ptr;

//#################### FUNCTIONS ####################
Descriptor_CPtr make_2d_descriptor(float x, float y)
{
  Descriptor_Ptr d(new Descriptor(2));
  (*d)[0] = x;
  (*d)[1] = y;
  return d;
}

std::vector<Descriptor_CPtr> generate_2d_descriptors_in_range(float min, float max, float resolution)
{
  std::vector<Descriptor_CPtr> descriptors;
  for(float x = min; x < max; x+=resolution)
  {
    for(float y = min; y < max; y+=resolution)
    {
      descriptors.push_back(make_2d_descriptor(x,y));
    }
  }
  return descriptors;
}

/**
 * \brief Gets an answer to the questions posed to the random forest.
 *
 * \param examples      The set of examples.
 * \return              The answer to the questions as quantified by a performance measure.
 */
std::map<std::string,evaluation::PerformanceMeasure> evaluate_forest_on_accuracy(const RandomForest_Ptr& forest, const std::vector<Example_CPtr>& examples)
{
  std::set<Label> classLabels;
  size_t examplesSize = examples.size();
  std::vector<Label> expectedLabels(examplesSize), predictedLabels(examplesSize);

  for(size_t i = 0; i < examplesSize; ++i)
  {
    const Example_CPtr& example = examples[i];
    predictedLabels[i] = forest->predict(example->get_descriptor());
    expectedLabels[i] = example->get_label();
    classLabels.insert(expectedLabels[i]);
  }

  Eigen::MatrixXf confusionMatrix = evaluation::ConfusionMatrixUtil::make_confusion_matrix(classLabels, expectedLabels, predictedLabels);

  return boost::assign::map_list_of("Accuracy", evaluation::ConfusionMatrixUtil::calculate_accuracy(confusionMatrix));
}

template <typename T>
class SetSampler
{
private:
  tvgutil::RandomNumberGenerator m_rng;

public:
  SetSampler(unsigned int seed)
  :m_rng(seed)
  {}

  T get_sample(const std::set<T> set)
  {
    int randomIndex = m_rng.generate_int_from_uniform(0, set.size() - 1);
    typename std::set<T>::const_iterator it(set.begin());
    std::advance(it, randomIndex);
    return *it;
  }
};

int main(int argc, char *argv[])
{
  // Set a seed for the random number generator.
  const unsigned int seed = 1234;

  // Generate a set of labels.
  const size_t labelCount = 20;
  std::set<Label> classLabels;
  for(size_t i = 0; i < labelCount; ++i) classLabels.insert(i);

  // Initialise the set sampler.
  SetSampler<Label> classLabelSampler(seed);

  // Generate a subset of labels which are currently observable;
  const size_t currentLabelCount = 2;
  std::set<Label> currentClassLabels;
  for(size_t i = 0; i < currentLabelCount; ++i) currentClassLabels.insert(i);

  // Generate a palette of random colours.
  std::map<Label,cv::Scalar> randomPalette = PaletteGenerator::generate_random_rgba_palette<Label>(classLabels, seed);

  // Generate a palette of basic colours.
  std::map<std::string,cv::Scalar> basicPalette = PaletteGenerator::generate_basic_rgba_palette();

  // Initialise a unit example generator.
  const float lowerSTD = 0.05f;
  const float upperSTD = 0.18f;
  rafl::UnitCircleExampleGenerator<Label> uceg(classLabels, seed, lowerSTD, upperSTD);

  // Generate the parameter sets with which to train the random forest.
  std::vector<ParamSet> params = CartesianProductParameterSetGenerator()
    .add_param("treeCount", list_of<size_t>(1))
    .add_param("splitBudget", list_of<size_t>(1048576))
    .add_param("candidateCount", list_of<int>(256))
    .add_param("decisionFunctionGeneratorType", list_of<std::string>("FeatureThresholding"))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(10000))
    .add_param("maxTreeHeight", list_of<size_t>(20))
    .add_param("randomSeed", list_of<unsigned int>(seed))
    .add_param("seenExamplesThreshold", list_of<size_t>(50))
    .add_param("splittabilityThreshold", list_of<float>(0.5f))
    .generate_param_sets();

  // Initialise the online random forest with the specified parameters.
  size_t treeCount  = 0;
  tvgutil::MapUtil::typed_lookup(params[0], "treeCount", treeCount);
  DecisionTree::Settings settings(params[0]);
  RandomForest_Ptr randomForest(new RandomForest(treeCount, settings));

  // Generate some figures used to display the output of the online learner.
  PlotWindow featureSpacePlot("UnitCircleExampleGenerator");

  PlotWindow decisionBoundaryPlot("DecisionBoundary");

  PlotWindow performancePlot("ClassificationAccuracy");

  // Set a specified time delay between learning iterations.
  int timeDelay = 20; //milliseconds
  const int maxDelay = 3000;

  // Create an OpenCV trackbar to vary the delay on the fly.
  cv::createTrackbar("Delay", "DecisionBoundary", &timeDelay, maxDelay);

  // Generate a dense set of points covering the 2d plane within a specified range.
  const float minVal = -2.5f;
  const float maxVal = 2.5f;
  const float resolution = 0.03f;
  std::vector<Descriptor_CPtr> pointsOnThePlane = generate_2d_descriptors_in_range(minVal, maxVal, resolution);

  // Initialise a vector to store the performance of the learner over time.
  std::vector<float> performanceOverTime;

  // Set the splitBudget of the random forest.
  size_t splitBudget = 0;
  tvgutil::MapUtil::typed_lookup(params[0], "splitBudget", splitBudget);

  // Set the maximum number of learning rounds.
  const size_t maxLearningRounds = 500;

  for(size_t roundCount = 0; roundCount < maxLearningRounds; ++roundCount)
  {
    if(roundCount % 20 == 0) currentClassLabels.insert(classLabelSampler.get_sample(classLabels));

    // Generate a set of examples from each class around the unit circle.
    std::vector<Example_CPtr> currentExamples = uceg.generate_examples(currentClassLabels, 50);

    // Draw the generated points in figure 1.
    for(size_t j = 1, jend = currentExamples.size(); j < jend; ++j)
    {
      const Example_CPtr& example = currentExamples[j];
      featureSpacePlot.draw_cartesian_circle(cv::Point2f((*example->get_descriptor())[0],(*example->get_descriptor())[1]), randomPalette[example->get_label()], 2, 2);
    }
    featureSpacePlot.draw_cartesian_axes(basicPalette["Red"]);
    featureSpacePlot.refresh();

    // Pass the set of examples to the random forest.
    randomForest->add_examples(currentExamples);

    // Predict the labels of the examples passed to the forest under the current hypothesis.
    std::map<std::string,evaluation::PerformanceMeasure> performance = evaluate_forest_on_accuracy(randomForest, currentExamples);
    performanceOverTime.push_back(performance.find("Accuracy")->second.get_mean());

    // Update the random forest to minimise errors on future predictions.
    randomForest->train(splitBudget);

    // Plot a line graph showing the performance of the forest over time.
    performancePlot.line_graph(performanceOverTime, basicPalette["Blue"]);

    // Draw the performance on the same figure as the line graph.
    boost::format threeDecimalPlaces("%0.3f");
    std::string currentPerformance = (threeDecimalPlaces % performanceOverTime.back()).str();
    std::string cumulativeAveragePerformance = (threeDecimalPlaces % static_cast<float>(std::accumulate(performanceOverTime.begin(), performanceOverTime.end(), 0.0f)/performanceOverTime.size())).str();
    performancePlot.draw_image_text( std::string("Cur") + (performance.find("Accuracy")->first) + currentPerformance, cv::Point2i(10, performancePlot.canvas_height() - 50), basicPalette["White"]);
    performancePlot.draw_image_text( std::string("Avg") + (performance.find("Accuracy")->first) + cumulativeAveragePerformance, cv::Point2i(10, performancePlot.canvas_height() - 10), basicPalette["White"]);
    performancePlot.refresh();

    // Draw the current decision boundary of the random forest by predicting the labels of the dense set of points generated in the 2D plane.
    for(int j = 1, jend = pointsOnThePlane.size(); j < jend; ++j)
    {
      Descriptor_CPtr descriptor = pointsOnThePlane[j];
      decisionBoundaryPlot.draw_cartesian_circle(cv::Point2f((*descriptor)[0],(*descriptor)[1]), randomPalette[randomForest->predict(descriptor)], 2, 2);
    }
    decisionBoundaryPlot.refresh();

    // Wait for keyboard events.
    if(cv::waitKey(timeDelay) == 'q') break;

    // Clear relevant figures.
    featureSpacePlot.clear_figure();
    performancePlot.clear_figure();

    // Remove class labels from the current label set.
    if(roundCount % 40 == 0) currentClassLabels.erase( classLabelSampler.get_sample(classLabels) );
  }

  return 0;
}

