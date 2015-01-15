//###
#if 0

#include <iostream>

#include <rafl/examples/ExampleReservoir.h>
using namespace rafl;

#include <rafl/DecisionTree.h>
using namespace rafl;
using namespace tvgutil;

int main()
{
  unsigned int seed = 0;
  ExampleReservoir<int> reservoir(10, RandomNumberGenerator_Ptr(new RandomNumberGenerator(seed)));
  for(int i = 0; i < 50; ++i)
  {
    typedef boost::shared_ptr<Example<int> > Example_CPtr;
    reservoir.add_example(Example_CPtr(new Example<int>(Descriptor_CPtr(), i)));
    std::cout << reservoir << '\n';
  }
  //DecisionTree<int> dt(10, RandomNumberGenerator_Ptr(new RandomNumberGenerator(seed)));
  return 0;
}

#endif

//###
#if 0

#include <iostream>

#include <rafl/base/ProbabilityMassFunction.h>
using namespace rafl;

int main()
{
  Histogram<int> hist;
  for(int i=0; i<5; ++i) hist.add(23);
  for(int i=0; i<5; ++i) hist.add(9);
  for(int i=0; i<5; ++i) hist.add(84);
  for(int i=0; i<500; ++i) hist.add(24);
  ProbabilityMassFunction<int> pmf(hist);
  const std::map<int,float>& masses = pmf.get_masses();
  /*for(std::map<int,float>::const_iterator it = masses.begin(), iend = masses.end(); it != iend; ++it)
  {
    std::cout << it->first << ' ' << it->second << '\n';
  }*/
  //std::cout << pmf << '\n';
  std::cout << pmf << '\n';
  std::cout << "Entropy=" << pmf.calculate_entropy() << '\n';
  return 0;
}

#endif

//###
#if 0

#include <rafl/DecisionTree.h>
#include <rafl/decisionfunctions/FeatureThresholdingDecisionFunctionGenerator.h>
using namespace rafl;

enum Label
{
  RED,
  BLUE
};

typedef DecisionTree<Label> DT;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

Descriptor_CPtr make_descriptor(float x, float y)
{
  Descriptor_Ptr d(new Descriptor(2));
  (*d)[0] = x;
  (*d)[1] = y;
  return d;
}

Example_CPtr make_example(float x, float y, Label l)
{
  return Example_CPtr(new Example<Label>(make_descriptor(x, y), l));
}

int main()
{
  unsigned int seed = 12345;
  tvgutil::RandomNumberGenerator_Ptr randomNumberGenerator(new tvgutil::RandomNumberGenerator(seed));
  DT::DecisionFunctionGenerator_CPtr decisionFunctionGenerator(new FeatureThresholdingDecisionFunctionGenerator<Label>(randomNumberGenerator));
  DT dt(10, 2, randomNumberGenerator, decisionFunctionGenerator);

  std::vector<Example_CPtr> examples;
  examples.push_back(make_example(0, 4, RED));
  examples.push_back(make_example(0, 5, RED));
  examples.push_back(make_example(1, 5, RED));
  examples.push_back(make_example(4, 0, BLUE));
  examples.push_back(make_example(5, 0, BLUE));
  examples.push_back(make_example(5, 1, BLUE));

  dt.add_examples(examples);
  dt.train(4);
  dt.output(std::cout);

  ProbabilityMassFunction<Label> pmf = dt.lookup_pmf(make_descriptor(0, 8));
  std::cout << pmf << '\n';
  return 0;
}

#endif

//###
#if 0

#include <Eigen/Dense>

#include <rafl/evaluation/PerformanceEvaluation.h>
#include <rafl/evaluation/CrossValidation.h>
#include <rafl/RandomForest.h>
#include <rafl/decisionfunctions/FeatureThresholdingDecisionFunctionGenerator.h>
using namespace rafl;

typedef int Label;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
typedef DecisionTree<Label> DT;
typedef RandomForest<Label> RF;
typedef boost::shared_ptr<DT::Settings> Settings_Ptr;

Descriptor_CPtr make_descriptor(float *arr)
{
  Descriptor_Ptr d(new Descriptor(10));
  for(int i = 0; i < 10; ++i)
  {
    (*d)[i] = arr[i];
  }
  return d;
}

int main(int argc, char *argv[])
{
  if(argc != 3)
  {
    std::cout << "Silly boy - enter the path of the training set, followed by the path of the test set!\n";
  }
  else
  {
    std::string trainingSetPath = argv[1];
    std::string testingSetPath = argv[2];

    std::cout << "Training set: " << trainingSetPath << "\n";
    std::cout << "Testing set: " << testingSetPath << "\n";

    // Construct the random forest.
    Settings_Ptr settings;
    try
    {
      settings.reset(new DT::Settings("settings.txt"));
    }
    catch(std::exception&)
    {
      // If the settings file can't be found or is invalid, use default settings.
      unsigned int seed = 12345;
      tvgutil::RandomNumberGenerator_Ptr randomNumberGenerator(new tvgutil::RandomNumberGenerator(seed));
      DT::DecisionFunctionGenerator_CPtr decisionFunctionGenerator(new FeatureThresholdingDecisionFunctionGenerator<Label>(randomNumberGenerator));

      settings.reset(new DT::Settings);
      settings->candidateCount = 256;
      settings->decisionFunctionGenerator = decisionFunctionGenerator;
      settings->gainThreshold = 0.0f;
      settings->maxClassSize = 10000;
      settings->maxTreeHeight = 15;
      settings->randomNumberGenerator = randomNumberGenerator;
      settings->seenExamplesThreshold = 30;
      settings->splittabilityThreshold = 0.5f;
    }

    const size_t treeCount = 8;
    RF rf(treeCount, *settings);

    // Train the random forest.
    std::cout << "Training the random forest..\n";
    std::vector<Example_CPtr> trainingExamples = ExampleUtil::load_examples<Label>(trainingSetPath);
    std::set<Label> classLabels;
    for(size_t i = 0, iend = trainingExamples.size(); i < iend; ++i)
    {
      classLabels.insert( trainingExamples.at(i)->get_label() );
    }

    rf.add_examples(trainingExamples);
    const size_t splitBudget = 32768;
    rf.train(splitBudget);
    rf.output(std::cout);

    // Test the random forest and output the results.
    std::cout << "Testing the random forest..\n";
    std::vector<Example_CPtr> testingExamples = ExampleUtil::load_examples<Label>(testingSetPath);
    std::vector<Label> expectedLabels, predictedLabels;

    float totalTests = static_cast<float>(testingExamples.size());
    size_t correctTests = 0, wrongTests = 0;
    for(std::vector<Example_CPtr>::const_iterator it = testingExamples.begin(), iend = testingExamples.end(); it != iend; ++it)
    {
      const Descriptor_CPtr& descriptor = (*it)->get_descriptor();
      const Label& expectedLabel = (*it)->get_label();
      Label predictedLabel = rf.predict(descriptor);
      expectedLabels.push_back(expectedLabel);
      predictedLabels.push_back(predictedLabel);
      if(predictedLabel == expectedLabel) ++correctTests;
      else ++wrongTests;
    }

    std::cout << "Correct %: " << correctTests / totalTests << '\n';
    std::cout << "Wrong %: " << wrongTests / totalTests << '\n';

    Eigen::MatrixXf confMtx = PerfEval::get_conf_mtx(classLabels, expectedLabels, predictedLabels);
    std::cout << "confMtx: \n" << confMtx << "\n";
    std::cout << "Accuracy: \n" << PerfEval::get_accuracy(confMtx) << "\n";

    std::cout << "confMtxNormL1: \n" << PerfEval::normalise_rows_L1(confMtx) << "\n";
    std::cout << "AccuracyNormL1: \n" << PerfEval::get_accuracy(PerfEval::L1norm_mtx_rows(confMtx)) << "\n";

  }
  return 0;
}

#endif

#if 0

#include <boost/any.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>
using boost::assign::list_of;
using boost::spirit::hold_any;

#include <iostream>

int main()
{
  /*hold_any treeCount(1212);

  int treeCountNew = 0;

  treeCountNew = boost::any_cast<int>(treeCount);*/

  hold_any a1;
  a1 = std::string("Foo");
  std::cout << a1 << "\n";

  std::string v1 = boost::spirit::any_cast<std::string>(a1);
  std::cout << v1 << "\n";

  hold_any a2;
  a2 = 5;
  std::cout << a2 << "\n";

  int v2 = boost::spirit::any_cast<int>(a2);

  return 0;
}
#endif

#if 1

#include <boost/assign/list_of.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>
using boost::assign::list_of;
using boost::spirit::hold_any;

#include <Eigen/Dense>

#include <rafl/evaluation/PerformanceEvaluation.h>
#include <rafl/evaluation/CrossValidation.h>
#include <rafl/evaluation/ParameterStringGenerator.h>
#include <rafl/evaluation/RFOnlineLearner.h>
#include <rafl/examples/UnitCircleExampleGenerator.h>
using namespace rafl;

#include <tvgutil/RandomNumberGenerator.h>
#include <tvgutil/LimitedContainer.h>

typedef int Label;
typedef RFOnlineLearner<Label> RFO;
typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

typedef std::map<std::string,hold_any> ParamSet;

int main(int argc, char *argv[])
{
  if((argc != 1) && (argc != 3))
  {
    throw std::runtime_error("Silly boy - enter the path of the training set, followed by the path of the test set!");
  }
  
  std::vector<Example_CPtr> examples;

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
  }

  if(argc == 3)
  {
    std::string trainingSetPath = argv[1];
    std::string testingSetPath = argv[2];

    std::cout << "Training set: " << trainingSetPath << "\n";
    std::cout << "Testing set: " << testingSetPath << "\n";

    examples = ExampleUtil::load_examples<Label>(trainingSetPath);
    std::vector<Example_CPtr> testingExamples = ExampleUtil::load_examples<Label>(testingSetPath);

    examples.insert(examples.end(), testingExamples.begin(), testingExamples.end() );
    std::cout << "number of examples= " << examples.size() << "\n";
  }
  
  //Generate parameters of your algorithm.
  std::vector<ParamSet> params = ParameterStringGenerator()
    .add_param("treeCount", list_of<size_t>(1)(2)(3))
    .add_param("splitBudget", list_of<size_t>(32768))
    .add_param("candidateCount", list_of<int>(256))
    .add_param("decisionFunctionGeneratorName", list_of<std::string>("FeatureThresholding"))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(10000))
    .add_param("maxTreeHeight", list_of<size_t>(15))
    .add_param("randomSeed", list_of<unsigned int>(1234))
    .add_param("seenExamplesThreshold", list_of<size_t>(30))
    .add_param("splittabilityThreshold", list_of<float>(0.5f))
    .generate_maps();

  const size_t numFolds = 5;
  const unsigned int seed = 1234;
  boost::shared_ptr<RFO> randomAlgorithm;
  std::map<std::string,Result> Results;

  for(size_t n = 0, nend = params.size(); n < nend; ++n)
  {
    randomAlgorithm.reset( new RFO(params[n]) );
    CrossValidation<RFO,Result,Label> cv(numFolds, seed);
    Result cvResult = cv.run(randomAlgorithm, examples); 
    std::cout << "The cross-validation result after " << cv.num_folds() << " folds is: " << cvResult << std::endl;
    Results.insert(std::make_pair(ParameterStringGenerator::to_string(params[n]), cvResult));
    
  }

  std::cout << tvgutil::make_limited_container(Results, 5) << "\n";

  return 0;
}

#endif
