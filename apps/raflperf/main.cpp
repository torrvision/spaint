#include <cmath>

#if 0
#include <boost/asio/ip/host_name.hpp>
#endif
#include <boost/assign/list_of.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>
using boost::assign::list_of;
using boost::spirit::hold_any;

#include <Eigen/Dense>

#include <rafl/evaluation/EvaluationResults.h>
#include <rafl/evaluation/PerfUtil.h>
#include <rafl/evaluation/QuantitativePerformance.h>
#include <rafl/evaluation/CrossValidation.h>
#include <rafl/evaluation/RandomlyPermuteAndDivideValidation.h>
#include <rafl/evaluation/ParameterSetProductGenerator.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

#include <tvgutil/RandomNumberGenerator.h>
#include <tvgutil/LimitedContainer.h>

#include "RFOnlineLearner.h"

typedef int Label;
typedef RFOnlineLearner<Label> RFO;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

typedef ParameterSetProductGenerator::ParamSet ParamSet;

int main(int argc, char *argv[])
{
#if 0
  std::string hostName = boost::asio::ip::host_name();
  if(hostName == "ms-tvg-workstation") std::cout << "Hey Mike..\n";
#endif

  if((argc != 1) && (argc != 4))
  {
    throw std::runtime_error("Silly boy - enter the path of the training set, followed by the path of the test set, and the path to save the results!");
  }

  std::vector<Example_CPtr> examples;
  std::string outputResultPath;

  if(argc == 1)
  {
    //Generate a set of labels.
    std::set<Label> classLabels;
    classLabels.insert(1);
    classLabels.insert(3);
    classLabels.insert(5);
    classLabels.insert(7);

    //Generate examples around the unit circle.
    UnitCircleExampleGenerator<Label> uceg(classLabels, 1234);
    examples = uceg.generate_examples(classLabels, 100);
    outputResultPath = "myDummyResultPath.txt";
  }

  if(argc == 4)
  {
    std::string trainingSetPath = argv[1];
    std::string testingSetPath = argv[2];
    outputResultPath = argv[3];

    std::cout << "Training set: " << trainingSetPath << "\n";
    std::cout << "Testing set: " << testingSetPath << "\n";

    examples = ExampleUtil::load_examples<Label>(trainingSetPath);
    std::vector<Example_CPtr> testingExamples = ExampleUtil::load_examples<Label>(testingSetPath);

    examples.insert(examples.end(), testingExamples.begin(), testingExamples.end() );
    std::cout << "number of examples= " << examples.size() << "\n";
  }

  //Generate parameters of your algorithm.
  std::vector<ParamSet> params = ParameterSetProductGenerator()
    .add_param("treeCount", list_of<size_t>(1)(3)(5)(7)(9)(11)(13)(15)(17)(19)(21))
    .add_param("splitBudget", list_of<size_t>(static_cast<size_t>(pow(2,20))))
    .add_param("candidateCount", list_of<int>(256))
    .add_param("decisionFunctionGeneratorName", list_of<std::string>("FeatureThresholding"))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(10000))
    .add_param("maxTreeHeight", list_of<size_t>(20))
    .add_param("randomSeed", list_of<unsigned int>(1234))
    .add_param("seenExamplesThreshold", list_of<size_t>(50))
    .add_param("splittabilityThreshold", list_of<float>(0.5f))
    .generate_maps();

  const size_t numFolds = 2;
  const unsigned int seed = 1234;
  boost::shared_ptr<RFO> randomAlgorithm;
  EvaluationResults results;

  for(size_t n = 0, nend = params.size(); n < nend; ++n)
  {
    randomAlgorithm.reset( new RFO(params[n]) );
    RandomlyPermuteAndDivideValidation<RFO,QuantitativePerformance,Label> rpadv(0.5f, 5, seed);
    QuantitativePerformance performance = rpadv.run(randomAlgorithm, examples);
    std::cout << "The randomly-permute-and-divide-validation result after " << rpadv.num_folds() << " folds is: " << performance << std::endl;
#if 0
    CrossValidation<RFO,QuantitativePerformance,Label> cv(numFolds, seed);
    QuantitativePerformance performance = cv.run(randomAlgorithm, examples);
    std::cout << "The cross-validation result after " << cv.num_folds() << " folds is: " << performance << std::endl;
#endif
    results.push_back(params[n], performance);
  }

  //Output results to the screen.
  results.print_tab_delimited(std::cout);

  //Output results to a file.
  std::ofstream resultsFile(outputResultPath.c_str());
  if(!resultsFile)
  {
    std::cout << "Warning could not open file for writing..\n";
  }
  else
  {
    results.print_tab_delimited(resultsFile);
  }

  return 0;
}
