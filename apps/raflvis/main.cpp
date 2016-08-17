/**
 * raflvis: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
using boost::assign::list_of;

#include <evaluation/core/ParamSetUtil.h>
#include <evaluation/core/PerformanceMeasure.h>
#include <evaluation/paramsetgenerators/CartesianProductParameterSetGenerator.h>
#include <evaluation/util/ConfusionMatrixUtil.h>
using namespace evaluation;

#include <rafl/choppers/HeightLimitingTreeChopper.h>
#include <rafl/choppers/TimeBasedTreeChopper.h>
#include <rafl/core/RandomForest.h>
#include <rafl/examples/ExampleUtil.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

#include <tvgplot/PaletteGenerator.h>
#include <tvgplot/PlotWindow.h>
using namespace tvgplot;

#include <tvgutil/persistence/SerializationUtil.h>
using namespace tvgutil;

#include "TestDecisionFunctionGenerator.h"

//#################### TYPEDEFS ####################

typedef int Label;

typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

typedef DecisionTree<Label> DT;
typedef RandomForest<Label> RF;
typedef boost::shared_ptr<RF> RF_Ptr;

typedef boost::shared_ptr<const TreeChopper<Label> > TreeChopper_CPtr;

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
 * \brief Rotates a set of 2D examples by a specified angle.
 *
 * \param examples   The set of examples.
 * \param angle      The angle by which to rotate each example (in radians).
 * \return           The rotated set of examples.
 */
static std::vector<Example_CPtr> rotate_examples(const std::vector<Example_CPtr> examples, float angle)
{
  std::vector<Example_CPtr> rotatedExamples;
  for(size_t i = 0, size = examples.size(); i < size; ++i)
  {
    const Descriptor_CPtr& desc = examples[i]->get_descriptor();
    const float x = (*desc)[0], y = (*desc)[1];
    const float rx = x * cosf(angle) - y * sinf(angle);
    const float ry = x * sinf(angle) + y * cosf(angle);
    rotatedExamples.push_back(Example_CPtr(new Example<Label>(make_2d_descriptor(rx,ry), examples[i]->get_label())));
  }
  return rotatedExamples;
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
//#define CLASS_IMBALANCE_TEST
#define EXAMPLE_ROTATION_TEST

  // The seed for the random number generator.
  const unsigned int seed = 1234;

  // Generate a set of labels.
#if defined(CLASS_IMBALANCE_TEST)
  const int labelCount = 5;
#elif defined(EXAMPLE_ROTATION_TEST)
  const int labelCount = 5;
#else
  const int labelCount = 20;
#endif
  std::set<Label> classLabels;
  for(int i = 0; i < labelCount; ++i) classLabels.insert(i);

#ifndef CLASS_IMBALANCE_TEST
  // Initialise the set sampler.
  SetSampler<Label> classLabelSampler(seed);
#endif

  // Generate a subset of the labels that are currently observable.
#ifdef CLASS_IMBALANCE_TEST
  std::set<Label> unbiasedClassLabels;
  for(int i = 0; i < labelCount - 1; ++i) unbiasedClassLabels.insert(i);
  std::set<Label> biasedClassLabels;
  biasedClassLabels.insert(labelCount - 1);
#else
  const int currentLabelCount = 2;
  std::set<Label> currentClassLabels;
  for(int i = 0; i < currentLabelCount; ++i) currentClassLabels.insert(i);
#endif

  // Generate the palettes.
  std::map<std::string,cv::Scalar> basicPalette = PaletteGenerator::generate_basic_rgba_palette();
  std::map<Label,cv::Scalar> randomPalette = PaletteGenerator::generate_random_rgba_palette<Label>(classLabels, seed);

  // Initialise a unit example generator.
  const float lowerSTD = 0.05f;
  const float upperSTD = 0.18f;
  UnitCircleExampleGenerator<Label> uceg(classLabels, seed, lowerSTD, upperSTD);

  // Generate the parameter set with which to train the random forest (note that we're using the parameter set generator for convenience only).
  std::vector<ParamSet> params = CartesianProductParameterSetGenerator()
    .add_param("candidateCount", list_of<int>(256))
    .add_param("decisionFunctionGeneratorParams", list_of<std::string>(""))
    .add_param("decisionFunctionGeneratorType", list_of<std::string>(PairwiseOpAndThresholdDecisionFunctionGenerator<Label>::get_static_type()))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(10000))
    .add_param("maxTreeHeight", list_of<size_t>(20))
    .add_param("randomSeed", list_of<unsigned int>(seed))
    .add_param("seenExamplesThreshold", list_of<size_t>(50))
    .add_param("splittabilityThreshold", list_of<float>(0.5f))
    .add_param("usePMFReweighting", list_of<bool>(true))
    .generate_param_sets();

  // Register the relevant decision function generators with the factory.
  DecisionFunctionGeneratorFactory<Label>::instance().register_rafl_makers();

  // Initialise the online random forest with the specified parameters.
  const size_t treeCount = 2;
  DT::Settings settings(params[0]);
  RF_Ptr randomForest(new RF(treeCount, settings));

  // Create the tree chopper.
  const size_t maxTreeHeight = 5;
  const size_t timePeriod = 20;
  TimeBasedTreeChopper<Label> treeChopper(TreeChopper_CPtr(new HeightLimitingTreeChopper<Label>(maxTreeHeight, seed)), timePeriod);

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
#if defined(CLASS_IMBALANCE_TEST)
    // Generate a set of examples with an unbalanced number of class labels.
    std::vector<Example_CPtr> currentExamples = uceg.generate_examples(unbiasedClassLabels, 30);
    std::vector<Example_CPtr> currentBiasedExamples = uceg.generate_examples(biasedClassLabels, 3000);
    currentExamples.insert(currentExamples.end(), currentBiasedExamples.begin(), currentBiasedExamples.end());
#elif defined(EXAMPLE_ROTATION_TEST)
    // Rotate the examples by an angle that increases by 10 degrees every 20 rounds until it hits a maximum of 90 degrees.
    static int degreeAngle = 0;
    if(roundCount % 20 == 0) degreeAngle = std::min(90, degreeAngle + 10);
    float radianAngle = static_cast<float>(degreeAngle * M_PI / 180.0);
    std::vector<Example_CPtr> currentExamples = rotate_examples(uceg.generate_examples(classLabels, 50), radianAngle);

    // Chop old trees in the forest as necessary to make the forest adapt better to the new examples as the rotation increases.
    treeChopper.chop_tree_if_necessary(randomForest);
#else
    // Add an additional current class label after every 20 rounds of training.
    if(roundCount % 20 == 0) currentClassLabels.insert(classLabelSampler.get_sample(classLabels));

    // Generate a set of examples from each current class around the unit circle.
    std::vector<Example_CPtr> currentExamples = uceg.generate_examples(currentClassLabels, 50);
#endif

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

    // Train the random forest by splitting up to the specified number of leaf nodes, and output forest statistics each time the forest
    // has split some nodes in order to help diagnose the learning.
    static const size_t splitBudget = 1;
    if(randomForest->train(splitBudget)) randomForest->output_statistics(std::cout);

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

#ifndef CLASS_IMBALANCE_TEST
    // Remove a current class label after every 40 rounds of training.
    if(roundCount % 40 == 0) currentClassLabels.erase(classLabelSampler.get_sample(classLabels));
#endif

    if(roundCount % 50 == 0)
    {
      const std::string path = "./randomForest" + boost::lexical_cast<std::string>(roundCount) + ".rf";
      SerializationUtil::save_text(path, *randomForest);
      randomForest = SerializationUtil::load_text(path, randomForest);
    }
  }

#undef CLASS_IMBALANCE_TEST

  return 0;
}
