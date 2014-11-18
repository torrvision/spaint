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

#if 1

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
  DT dt(10, randomNumberGenerator, decisionFunctionGenerator);

  std::vector<Example_CPtr> examples;
  examples.push_back(make_example(0, 4, RED));
  examples.push_back(make_example(0, 5, RED));
  examples.push_back(make_example(1, 5, RED));
  examples.push_back(make_example(4, 0, BLUE));
  examples.push_back(make_example(5, 0, BLUE));
  examples.push_back(make_example(5, 1, BLUE));

  dt.add_examples(examples);
  dt.train(4);

  ProbabilityMassFunction<Label> pmf = dt.lookup_pmf(make_descriptor(0, 8));
  std::cout << pmf << '\n';
  
  
  std::set<Label> labelSet;
  labelSet.insert(RED);
  labelSet.insert(BLUE);
  const size_t numberOfExamplesPerClass = 5;
  std::vector<Example_CPtr> examplesOnUnitCircle = ExampleUtil::unit_circle_example_generator(labelSet, numberOfExamplesPerClass);
  
  return 0;
}

#endif
