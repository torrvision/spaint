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
#if 1

#include <Eigen/Dense>

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
      DT::DecisionFunctionGenerator_CPtr decisionFunctionGenerator(new FeatureThresholdingDecisionFunctionGenerator<Label>);

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
  }
  return 0;
}

#endif
