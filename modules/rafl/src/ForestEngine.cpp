#include "ForestEngine.h"

#include <iostream>

namespace rafl {

ForestEngine::ForestEngine(int a)
: m_a(a)
{
  std::cout << "Hi I'm ForestEngine and my value is:" << m_a << std::endl;
}

}
