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

#include <tvgutil/colours/PaletteGenerator.h>
#include "CvMatPlot.h"
#include "OnlineRandomForestLearner.h"

//#################### TYPEDEFS ####################

typedef int Label;
typedef boost::shared_ptr<const Descriptor> Descriptor_CPtr;
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

int main(int argc, char *argv[])
{
  const unsigned int seed = 1234;

  std::vector<Example_CPtr> examples;
  std::string outputResultPath;

  // Generate a set of labels.
  std::set<Label> classLabels;
  classLabels.insert(1);
  classLabels.insert(3);
  classLabels.insert(5);
  classLabels.insert(7);

  std::map<Label,cv::Scalar> palette = tvgutil::PaletteGenerator::generate_random_rgba_palette<Label,cv::Scalar>(classLabels, 1234);

  // Generate examples around the unit circle.
  UnitCircleExampleGenerator<Label> uceg(classLabels, seed);

  // Generate the parameter sets with which to test the random forest.
  std::vector<ParamSet> params = CartesianProductParameterSetGenerator()
    .add_param("treeCount", list_of<size_t>(1))
    .add_param("splitBudget", list_of<size_t>(1024)(1048576))
    .add_param("candidateCount", list_of<int>(256))
    .add_param("decisionFunctionGeneratorType", list_of<std::string>("FeatureThresholding"))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(10000))
    .add_param("maxTreeHeight", list_of<size_t>(20))
    .add_param("randomSeed", list_of<unsigned int>(seed))
    .add_param("seenExamplesThreshold", list_of<size_t>(50))
    .add_param("splittabilityThreshold", list_of<float>(0.5f))
    .generate_param_sets();

  OnlineRandomForestLearner<Label> orfl(params[0]);

  CvMatPlot fig1(1, "UnitCircleExampleGenerator");
  CvMatPlot fig2(2, "DecisionBoundary");
  CvMatPlot fig3(3, "ClassificationAccuracy");

  // Generate points on the 2d plane
  std::vector<Descriptor_CPtr> pointsOnThePlane = generate_2d_descriptors_in_range(-2.0f, 2.0f, 0.1f);

  std::vector<float> performanceOverTime;

  const size_t maxIterations = 100;
  for(size_t i = 0; i < maxIterations; ++i)
  {
    std::vector<Example_CPtr> currentExamples = uceg.generate_examples(classLabels,50);
    for(int j = 1, jend = currentExamples.size(); j < jend; ++j)
    {
      Example_CPtr example = currentExamples[j];
      fig1.canvas_point(cv::Point2f((*example->get_descriptor())[0],(*example->get_descriptor())[1]), palette[example->get_label()], 2, 2);
    }
    fig1.show();

    // Online learning.
    orfl.question(currentExamples);
    std::map<std::string,evaluation::PerformanceMeasure> performance = orfl.answer(currentExamples);
    /*
    orfl.update();
    */

/*
    // Plot bar chart.
    performanceOverTime.push_back(performance.find("Accuracy")->second.get_mean());
    fig3.draw_bars(performanceOverTime, cv::Scalar(0,0,255));
    fig3.show();

    for(int j = 1, jend = pointsOnThePlane.size(); j < jend; ++j)
    {
      Descriptor_CPtr descriptor = pointsOnThePlane[j];
      fig2.canvas_point(cv::Point2f((*descriptor)[0],(*descriptor)[1]), palette[orfl.predict(descriptor)], 2, 2);
    }
    fig2.show();
*/
    cv::waitKey(50);
    //fig1.clf();
    //fig2.clf();
  }

#if 0  
  outputResultPath = "UnitCircleExampleGenerator-Results.txt";

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
#endif

  return 0;
}
