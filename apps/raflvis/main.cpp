/**
 * raflvis: main.cpp
 */

#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
using boost::assign::list_of;

#include <evaluation/util/CartesianProductParameterSetGenerator.h>
#include <evaluation/util/ConfusionMatrixUtil.h>
#include <evaluation/core/PerformanceMeasure.h>
using namespace evaluation;

#include <rafl/RandomForest.h>
#include <rafl/examples/ExampleUtil.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

#include "PaletteGenerator.h"
#include "PlotWindow.h"

//#################### TYPEDEFS ####################

typedef int Label;

typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

typedef DecisionTree<Label> DT;
typedef RandomForest<Label> RF;
typedef boost::shared_ptr<RF> RF_Ptr;

typedef CartesianProductParameterSetGenerator::ParamSet ParamSet;

//#################### TYPES ####################

/**
 * \brief An instance of this class can be used to pick random elements from a set.
 */
template <typename T>
class SetSampler
{
  //~~~~~~~~~~~~~~~~~~~~ PRIVATE VARIABLES ~~~~~~~~~~~~~~~~~~~~
private:
  /** The random number generator used to generate random indices for sampling. */
  tvgutil::RandomNumberGenerator m_rng;

  //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~
public:
  /**
   * \brief Constructs a set sampler.
   *
   * \param seed  The seed for the random number generator.
   */
  explicit SetSampler(unsigned int seed)
  : m_rng(seed)
  {}

  //~~~~~~~~~~~~~~~~~~~~ PUBLIC MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~
public:
  /**
   * \brief Picks a random element from the specified set.
   *
   * \param s The set from which to sample.
   * \return  The sampled element.
   */
  T get_sample(const std::set<T>& s)
  {
    int randomIndex = m_rng.generate_int_from_uniform(0, static_cast<int>(s.size()) - 1);
    typename std::set<T>::const_iterator it(s.begin());
    std::advance(it, randomIndex);
    return *it;
  }
};

//#################### FUNCTIONS ####################

/**
 * \brief Makes a 2D descriptor.
 *
 * \param x The x component of the descriptor.
 * \param y The y component of the descriptor.
 * \return  The descriptor.
 */
static Descriptor_CPtr make_2d_descriptor(float x, float y)
{
  Descriptor_Ptr d(new Descriptor(2));
  (*d)[0] = x;
  (*d)[1] = y;
  return d;
}

/**
 * \brief Generates a grid of 2D descriptors over the (min,min)-(max,max) bounding box, with the specified spacing.
 *
 * \param min     The minimum coordinate in the box in both the x and y directions.
 * \param max     The maximum coordinate in the box in both the x and y directions.
 * \param spacing The spacing between the individual descriptors.
 * \return        The grid of descriptors.
 */
static std::vector<Descriptor_CPtr> generate_2d_descriptor_grid(float min, float max, float spacing)
{
  std::vector<Descriptor_CPtr> descriptors;
  for(float x = min; x < max; x += spacing)
  {
    for(float y = min; y < max; y += spacing)
    {
      descriptors.push_back(make_2d_descriptor(x, y));
    }
  }
  return descriptors;
}

/**
 * \brief Evaluates the accuracy of the random forest on a set of examples.
 *
 * \param forest    The random forest.
 * \param examples  The set of examples.
 * \return          The accuracy of the random forest on the examples.
 */
static float evaluate_forest_accuracy(const RF_Ptr& forest, const std::vector<Example_CPtr>& examples)
{
  std::set<Label> classLabels;
  size_t examplesSize = examples.size();
  std::vector<Label> expectedLabels(examplesSize), predictedLabels(examplesSize);

  for(size_t i = 0; i < examplesSize; ++i)
  {
    const Example<Label>& example = *examples[i];
    predictedLabels[i] = forest->predict(example.get_descriptor());
    expectedLabels[i] = example.get_label();
    classLabels.insert(expectedLabels[i]);
  }

  Eigen::MatrixXf confusionMatrix = ConfusionMatrixUtil::make_confusion_matrix(classLabels, expectedLabels, predictedLabels);
  return ConfusionMatrixUtil::calculate_accuracy(ConfusionMatrixUtil::normalise_rows_L1(confusionMatrix));
}

int main(int argc, char *argv[])
{
  // The seed for the random number generator.
  const unsigned int seed = 1234;

  // Generate a set of labels.
  const int labelCount = 3;
  std::set<Label> classLabels;
  for(int i = 0; i < labelCount; ++i) classLabels.insert(i);

  // Generate a subset of the labels that are currently observable.
  std::set<Label> unbiasedClassLabels;
  for(int i = 0; i < (labelCount - 1); ++i) unbiasedClassLabels.insert(i);
  std::set<Label> biasedClassLabels;
  biasedClassLabels.insert(labelCount - 1);

  // Generate the palettes.
  std::map<std::string,cv::Scalar> basicPalette = PaletteGenerator::generate_basic_rgba_palette();
  std::map<Label,cv::Scalar> randomPalette = PaletteGenerator::generate_random_rgba_palette<Label>(classLabels, seed);

  // Initialise a unit example generator.
  const float lowerSTD = 0.05f;
  const float upperSTD = 0.18f;
  UnitCircleExampleGenerator<Label> uceg(classLabels, seed, lowerSTD, upperSTD);

  // Generate the parameter set with which to train the random forest (note that we're using the parameter set generator for convenience only).
  std::vector<ParamSet> params = CartesianProductParameterSetGenerator()
    .add_param("candidateCount", list_of<int>(128))
    .add_param("decisionFunctionGeneratorType", list_of<std::string>("FeatureThresholding"))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(1000))
    .add_param("maxTreeHeight", list_of<size_t>(20))
    .add_param("randomSeed", list_of<unsigned int>(seed))
    .add_param("reweight", list_of<bool>(false))
    .add_param("seenExamplesThreshold", list_of<size_t>(50))
    .add_param("splittabilityThreshold", list_of<float>(0.5f))
    .generate_param_sets();

  // Initialise the online random forest with the specified parameters.
  const size_t treeCount = 1;
  DT::Settings settings(params[0]);
  RF_Ptr randomForest(new RF(treeCount, settings));

  // Generate the windows into which we will display the output of the random forest.
  PlotWindow accuracyPlot("ClassificationAccuracy");
  PlotWindow decisionBoundaryPlot("DecisionBoundary");
  PlotWindow featureSpacePlot("UnitCircleExampleGenerator");

  // Set the initial and maximum time delays between learning iterations (in milliseconds).
  int timeDelay = 20;
  const int maxDelay = 3000;

  // Create an OpenCV trackbar to allow the user to vary the time delay on the fly.
  cv::createTrackbar("Delay", "DecisionBoundary", &timeDelay, maxDelay);

  // Generate a dense set of points covering a square bounding box in the 2D plane.
  const float minVal = -2.5f;
  const float maxVal = 2.5f;
  const float spacing = 0.03f;
  std::vector<Descriptor_CPtr> descriptorGrid = generate_2d_descriptor_grid(minVal, maxVal, spacing);

  // Evaluate the accuracy of the random forest as we add new examples to it over a specified number of learning rounds, and visualise the results.
  std::vector<float> accuracyOverTime;
  const size_t maxLearningRounds = 500;
  for(size_t roundCount = 0; roundCount < maxLearningRounds; ++roundCount)
  {
    // Generate a set of examples from each current class around the unit circle.
    std::vector<Example_CPtr> currentExamples = uceg.generate_examples(unbiasedClassLabels, 30);
    std::vector<Example_CPtr> currentBiasedExamples = uceg.generate_examples(biasedClassLabels, 3000);

    currentExamples.insert(currentExamples.end(), currentBiasedExamples.begin(), currentBiasedExamples.end());

    // Draw the generated points in the feature space plot.
    for(size_t i = 1, iend = currentExamples.size(); i < iend; ++i)
    {
      const Example<Label>& example = *currentExamples[i];
      const Descriptor& descriptor = *example.get_descriptor();
      featureSpacePlot.draw_cartesian_circle(cv::Point2f(descriptor[0], descriptor[1]), randomPalette[example.get_label()], 2, 2);
    }
    featureSpacePlot.draw_cartesian_axes(basicPalette["Red"]);
    featureSpacePlot.refresh();

    // Add the set of examples to the random forest.
    randomForest->add_examples(currentExamples);

    // Predict the labels of the examples passed to the forest under the current hypothesis.
    accuracyOverTime.push_back(evaluate_forest_accuracy(randomForest, currentExamples));

    // Train the random forest by splitting up to the specified number of leaf nodes.
    static const size_t splitBudget = 1048576;
    randomForest->train(splitBudget);

    // Plot a line graph showing the accuracy of the forest on the training examples over time.
    accuracyPlot.draw_line_graph(accuracyOverTime, basicPalette["Blue"]);

    // Draw the accuracy values on the same plot as the line graph.
    boost::format threeDecimalPlaces("%0.3f");
    std::string currentAccuracy = (threeDecimalPlaces % accuracyOverTime.back()).str();
    std::string cumulativeAverageAccuracy = (threeDecimalPlaces % static_cast<float>(std::accumulate(accuracyOverTime.begin(), accuracyOverTime.end(), 0.0f) / accuracyOverTime.size())).str();
    accuracyPlot.draw_canvas_text( std::string("CurAccuracy") + currentAccuracy, cv::Point2i(10, accuracyPlot.canvas_height() - 50), basicPalette["White"]);
    accuracyPlot.draw_canvas_text( std::string("AvgAccuracy") + cumulativeAverageAccuracy, cv::Point2i(10, accuracyPlot.canvas_height() - 10), basicPalette["White"]);
    accuracyPlot.refresh();

    // Draw the current decision boundary of the random forest by predicting the labels of the dense set of points generated in the 2D plane.
    for(size_t i = 1, iend = descriptorGrid.size(); i < iend; ++i)
    {
      const Descriptor_CPtr& descriptor = descriptorGrid[i];
      decisionBoundaryPlot.draw_cartesian_circle(cv::Point2f((*descriptor)[0],(*descriptor)[1]), randomPalette[randomForest->predict(descriptor)], 2, 2);
    }
    decisionBoundaryPlot.refresh();

    // Process keyboard input.
    if(cv::waitKey(timeDelay) == 'q') break;

    // Clear the relevant plots.
    featureSpacePlot.clear_figure();
    accuracyPlot.clear_figure();
  }

  return 0;
}
