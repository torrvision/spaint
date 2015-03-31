#if 0
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <tvgutil/LimitedContainer.h>
#include <tvgutil/Serialization.h>

namespace ex {
class Example;
}

namespace boost { namespace serialization {
template<class Archive>
inline void save_construct_data(Archive& ar, const ex::Example *example, const unsigned int file_version);

template<class Archive>
inline void load_construct_data(Archive& ar, ex::Example *example, const unsigned int file_version);
}}


namespace ex {

class Example
{
private:
  int m_label;
  boost::shared_ptr<std::vector<float>> m_values;

public:
Example(boost::shared_ptr<std::vector<float>> values, int label)
: m_label(label), m_values(values)
{
  std::cout << "Constructing an example with:\n";
  std::cout << tvgutil::make_limited_container(*values,20) << '\n';
  std::cout << label << '\n';
  std::cout << tvgutil::make_limited_container(*m_values,20) << '\n';
  std::cout << m_label << '\n';
}

void print() const
{
  std::cout << "label = " << m_label << ", vec = ";
  for(size_t i = 0, iend = m_values->size(); i < iend; ++i)
  {
    std::cout << m_values->at(i) << ", ";
  }
  std::cout << "\n\n";
}

friend class boost::serialization::access;
template<typename Archive>
void serialize(Archive& ar, const unsigned int version)
{
  /* Note this has been left empty on purpose. */
}

template<class Archive>
friend void boost::serialization::save_construct_data(Archive& ar, const Example *example, const unsigned int file_version);

template<class Archive>
friend void boost::serialization::load_construct_data(Archive& ar, Example *example, const unsigned int file_version);
};

} // end namespace


namespace boost { namespace serialization {

template<class Archive>
inline void save_construct_data(Archive& ar, const ex::Example *example, const unsigned int file_version)
{
  std::cout << "saving the data\n";
  ar << example->m_label;
  ar << example->m_values;
}

template<class Archive>
inline void load_construct_data(Archive& ar, ex::Example *example, const unsigned int file_version)
{
  std::cout << "loading the data\n";

  int label;
  ar >> label;

  boost::shared_ptr<std::vector<float>> values;
  ar >> values;

  ::new(example)ex::Example(values, label);
}

}}

using namespace ex;
int main()
{
  boost::shared_ptr<std::vector<float>> vecPtr(new std::vector<float>);
  vecPtr->push_back(3.4);
  vecPtr->push_back(3.2);
  vecPtr->push_back(7.6);

  Example myexample(vecPtr, 1);
  myexample.print();
  tvgutil::boost_serial_save<Example>("./myexample.ex", &myexample);

  boost::shared_ptr<std::vector<float>> vecPtr2(new std::vector<float>);
  vecPtr2->push_back(5.4);
  vecPtr2->push_back(3.5);
  vecPtr2->push_back(5.6);

  Example *myexample2Ptr( new Example(vecPtr2, 30));
  myexample2Ptr->print();

  tvgutil::boost_serial_load<Example>("./myexample.ex", &myexample2Ptr);
  myexample2Ptr->print();

  return 0;
}
#endif

//###
#if 1

#include <iostream>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <rafl/base/ProbabilityMassFunction.h>
#include <rafl/examples/ExampleReservoir.h>
#include <rafl/decisionfunctions/FeatureThresholdingDecisionFunction.h>
#include <rafl/DecisionTree.h>

#include <evaluation/util/CartesianProductParameterSetGenerator.h>

#include <tvgutil/RandomNumberGenerator.h>
#include <tvgutil/Serialization.h>
#include <tvgutil/PriorityQueue.h>

using namespace rafl;
using namespace tvgutil;

typedef Histogram<int> HistogramS32;

Descriptor_CPtr make_descriptor(float *arr)
{
  Descriptor_Ptr d(new Descriptor(10));
  for(int i = 0; i < 10; ++i)
  {
    (*d)[i] = arr[i];
  }
  return d;
}

int main()
{
#if 0
  // Histogram Serialization.
  HistogramS32 hist;
  for(int i=0; i<5; ++i) hist.add(23);
  for(int i=0; i<5; ++i) hist.add(9);
  for(int i=0; i<5; ++i) hist.add(84);
  for(int i=0; i<500; ++i) hist.add(24);
  std::cout << hist << std::endl;

  std::string path = "./myhistogram.hist";
  boost_serial_save<HistogramS32>(path, &hist);

  HistogramS32 *hist2Ptr;
  boost_serial_load<HistogramS32>(path, &hist2Ptr);
  std::cout << *hist2Ptr << std::endl;

  ProbabilityMassFunction<int> pmf(*hist2Ptr);
  std::cout << pmf << '\n';
  std::cout << "Entropy=" << pmf.calculate_entropy() << '\n';
#endif

#if 0
  // Random number generator.
  RandomNumberGenerator rng(12345);
  boost_serial_save<tvgutil::RandomNumberGenerator>("./rng.rng", &rng);

  RandomNumberGenerator *rng2(new RandomNumberGenerator(1));
  boost_serial_load("./rng.rng", &rng2);
#endif

#if 0
  // Descriptor.
  Descriptor_Ptr descriptor = boost::shared_ptr<Descriptor>(new Descriptor);
  descriptor->push_back(1.2);
  descriptor->push_back(2);
  descriptor->push_back(5.4);
  Example<int> example(descriptor, 5);
  std::cout << example << std::endl;
  boost_serial_save<Example<int> >("./example.ex", &example);

  Descriptor_Ptr descriptor2 = boost::shared_ptr<Descriptor>(new Descriptor);
  descriptor2->push_back(10.5);
  descriptor2->push_back(1.9);
  descriptor2->push_back(8.1);
  Example<int> *examplePtr(new Example<int>(descriptor2, 3));
  std::cout << *examplePtr << std::endl;

  boost_serial_load<Example<int> >("./example.ex", &examplePtr);
  std::cout << *examplePtr << std::endl;
#endif

#if 0
  typedef boost::shared_ptr<const Example<int> > Example_CPtr;
  typedef boost::shared_ptr<Example<int> > Example_Ptr;

  //Map from Labels to example vectors.
  Descriptor_Ptr descriptor = boost::shared_ptr<Descriptor>(new Descriptor);
  descriptor->push_back(1.2);
  descriptor->push_back(2);
  descriptor->push_back(5.4);
  Example_Ptr example = Example_Ptr(new Example<int>(descriptor, 5));

  std::vector<Example_CPtr> examples;
  examples.push_back(example);

  std::map<int,std::vector<Example_CPtr> > exampleMap;
  exampleMap.insert(std::make_pair(5, examples));

  boost_serial_save<std::map<int,std::vector<Example_CPtr> > >("./exampleMap.em", &exampleMap);
  
  std::map<int,std::vector<Example_Ptr> > *exampleMapPtr;
  boost_serial_load<std::map<int,std::vector<Example_Ptr> > >("./exampleMap.em", &exampleMapPtr);

#endif

#if 0
  // Example Reservoir.
  const size_t maxClassSize = 1000;
  RandomNumberGenerator_Ptr rngPtr = RandomNumberGenerator_Ptr(new RandomNumberGenerator(1)); 
  ExampleReservoir<int> reservoir(maxClassSize, rngPtr);
  std::cout << "Current size = " << reservoir.current_size() << std::endl;
  boost_serial_save<ExampleReservoir<int>>("./examplereservoir.er", &reservoir);

  ExampleReservoir<int> *reservoir2(new ExampleReservoir<int>(maxClassSize, rngPtr));
  boost_serial_load<ExampleReservoir<int>>("./examplereservoir.er", &reservoir2);
  std::cout << "Current size = " << reservoir2->current_size() << std::endl;
#endif

#if 0
  // DecisionFunction.
  boost::shared_ptr<FeatureThresholdingDecisionFunction> decisionf( new FeatureThresholdingDecisionFunction(5, 0.3));
  boost_serial_save<boost::shared_ptr<FeatureThresholdingDecisionFunction> >("./ftdf.df", &decisionf);
  //std::cout << "featureIndex=" << decisionf->m_featureIndex << '\n';

  boost::shared_ptr<FeatureThresholdingDecisionFunction> *decisionf2;
  decisionf2->reset(new FeatureThresholdingDecisionFunction(7, 0.5));
  //std::cout << "featureIndex=" << decisionf2->get()->m_featureIndex << '\n';
  boost_serial_load<boost::shared_ptr<FeatureThresholdingDecisionFunction> >("./ftdf.df", &decisionf2);
  //std::cout << "featureIndex=" << decisionf2->get()->m_featureIndex << '\n';
#endif

#if 0
  //Node.
  RandomNumberGenerator_Ptr rngPtr = RandomNumberGenerator_Ptr(new RandomNumberGenerator(1)); 
  DecisionTree<int>::Node node1(5, 100, rngPtr);
  boost_serial_save<DecisionTree<int>::Node>("./node1.n", &node1);

  DecisionTree<int>::Node *node2( new DecisionTree<int>::Node(2, 10, rngPtr));
  boost_serial_load<DecisionTree<int>::Node>("./node1.n", &node2);
#endif

#if 0
  typedef evaluation::CartesianProductParameterSetGenerator::ParamSet ParamSet;
  //Settings.
  std::vector<ParamSet> params = evaluation::CartesianProductParameterSetGenerator()
    .add_param("candidateCount", list_of<int>(256))
    .add_param("decisionFunctionGeneratorType", list_of<std::string>("FeatureThresholding"))
    .add_param("gainThreshold", list_of<float>(0.0f))
    .add_param("maxClassSize", list_of<size_t>(10000))
    .add_param("maxTreeHeight", list_of<size_t>(20))
    .add_param("randomSeed", list_of<unsigned int>(1234))
    .add_param("seenExamplesThreshold", list_of<size_t>(50))
    .add_param("splittabilityThreshold", list_of<float>(0.5f))
    .generate_param_sets();

  DecisionTree<int>::Settings settings1(params[0]);

  boost_serial_save<DecisionTree<int>::Settings>("./settings1.rs", &settings1);
  std::cout << settings1.candidateCount << '\n';

  DecisionTree<int>::Settings *emptySettings;
  boost_serial_load<DecisionTree<int>::Settings>("./settings1.rs", &emptySettings);
  std::cout << emptySettings->candidateCount << '\n';
#endif

#if 1
  typedef tvgutil::PriorityQueue<int,float,void*,std::greater<float> > SplittabilityQueue;
  SplittabilityQueue myqueue;
  boost_serial_save<SplittabilityQueue>("./queue.pq", &myqueue);
#endif
  return 0;
}

#endif

//###
#if 0

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
  }
  return 0;
}

#endif
