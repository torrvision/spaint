#include <iostream>

#include <tvgutil/RandomNumberGenerator.h>
using namespace tvgutil;

int main()
{
  RandomNumberGenerator rng(1234);

  for(int i = 0;i < 10; ++i)
  {
    std::cout << rng.generate_int_in_range(-8,8) << '\n';
  }

  return 0;
}
