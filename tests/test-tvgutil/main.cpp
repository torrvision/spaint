#include <iostream>

#include <tvgutil/RandomNumberGenerator.h>
using namespace tvgutil;

int main()
{
  RandomNumberGenerator randomNumberGenerator(1234);
  for(int i = 0; i < 10; ++i)
  {
    std::cout << randomNumberGenerator.generate_int_in_range(-8,8) << '\n';
  }
  return 0;
}
