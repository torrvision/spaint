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
  std::cout << "Entropy=" << pmf.get_entropy() << '\n';
  return 0;
}
