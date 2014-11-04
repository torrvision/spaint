#include <iostream>

#include <rafl/examples/ExampleReservoir.h>
using namespace rafl;

int main()
{
  unsigned int seed = 0;
  ExampleReservoir reservoir(10, seed);
  for(int i = 0; i < 20; ++i)
  {
    reservoir.add_example(i);
    std::cout << reservoir << '\n';
  }
  return 0;
}
