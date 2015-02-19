/**
 * raflvis: main.cpp
 */

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <evaluation/util/CartesianProductParameterSetGenerator.h>
using namespace evaluation;

#include <rafl/examples/ExampleUtil.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

#include <tvgutil/RandomNumberGenerator.h>

#include "CvPlotter.h"
#include "OnlineRandomForestLearner.h"
#include "PaletteGenerator.h"

//#################### TYPEDEFS ####################
typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
typedef CartesianProductParameterSetGenerator::ParamSet ParamSet;

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

  // Used OpenCV to detect keyboard input.
  static int key;

  // Generate a set of labels.
  const size_t labelCount = 20;
  std::set<Label> classLabels;
  for(size_t i = 0; i < labelCount; ++i) classLabels.insert(i);

  //Initialise the set sampler.
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
  UnitCircleExampleGenerator<Label> uceg(classLabels, seed, lowerSTD, upperSTD);

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
  OnlineRandomForestLearner<Label> orfl(params[0]);

  // Generate some figures used to display the output of the online learner.
  CvPlotter fig1(1, "UnitCircleExampleGenerator");

  CvPlotter fig2(2, "DecisionBoundary");

  CvPlotter fig3(3, "ClassificationAccuracy");

  // Generate a dense set of points covering the 2d plane within a specified range.
  const float minVal = -2.5f;
  const float maxVal = 2.5f;
  const float resolution = 0.03f;
  std::vector<Descriptor_CPtr> pointsOnThePlane = generate_2d_descriptors_in_range(minVal, maxVal, resolution);

  // Initialise a vector to store the performance of the learner over time.
  std::vector<float> performanceOverTime;

  const size_t maxLearningRounds = 500;

  for(size_t roundCount = 0; roundCount < maxLearningRounds; ++roundCount)
  {
    if((roundCount % 20) == 0) currentClassLabels.insert( classLabelSampler.get_sample(classLabels) );

    // Generate a set of examples from each class around the unit circle.
    //std::vector<Example_CPtr> currentExamples = uceg.generate_examples(classLabels, 50);
    std::vector<Example_CPtr> currentExamples = uceg.generate_examples(currentClassLabels, 50);

    // Draw the generated points in figure 1.
    for(int j = 1, jend = currentExamples.size(); j < jend; ++j)
    {
      Example_CPtr example = currentExamples[j];
      fig1.cartesian_point(cv::Point2f((*example->get_descriptor())[0],(*example->get_descriptor())[1]), randomPalette[example->get_label()], 2, 2);
    }
    fig1.cartesian_axes(basicPalette["Red"]);
    fig1.show();

    // Pass the set of examples to the random forest.
    orfl.question(currentExamples);

    // Predict the labels of the examples passed to the forest under the current hypothesis.
    std::map<std::string,evaluation::PerformanceMeasure> performance = orfl.answer(currentExamples);
    performanceOverTime.push_back(performance.find("Accuracy")->second.get_mean());

    // Update the random forest to minimise errors on future predictions.
    orfl.update();

    // Plot a line graph showing the performance of the forest over time.
    fig3.line_graph(performanceOverTime, basicPalette["Blue"]);

    // Draw the performance on the same figure as the line graph.
    char currentPerformance[7]; sprintf(currentPerformance, "%0.3f", performanceOverTime.back());
    char cumulativeAveragePerformance[7]; sprintf(cumulativeAveragePerformance, "%0.3f", static_cast<float>(std::accumulate(performanceOverTime.begin(), performanceOverTime.end(), 0.0f)/performanceOverTime.size()));
    fig3.image_text( std::string("Cur") + (performance.find("Accuracy")->first) + std::string(currentPerformance), cv::Point2i(10, fig3.height() - 50), basicPalette["White"]);
    fig3.image_text( std::string("Avg") + (performance.find("Accuracy")->first) + std::string(cumulativeAveragePerformance), cv::Point2i(10, fig3.height() - 10), basicPalette["White"]);
    fig3.show();

    // Draw the current decision boundary of the random forest by predicting the labels of the dense set of points generated in the 2D plane.
    for(int j = 1, jend = pointsOnThePlane.size(); j < jend; ++j)
    {
      Descriptor_CPtr descriptor = pointsOnThePlane[j];
      fig2.cartesian_point(cv::Point2f((*descriptor)[0],(*descriptor)[1]), randomPalette[orfl.predict(descriptor)], 2, 2);
    }
    fig2.show();

    // Wait for keyboard events.
    key = cv::waitKey(20);
    if(key == 'q') break;

    // Clear relevant figures.
    fig1.clf();
    fig3.clf();

    //Remove class labels from the current label set.
    if((roundCount % 40) == 0) currentClassLabels.erase( classLabelSampler.get_sample(classLabels) );
  }

  return 0;
}

