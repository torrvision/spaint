/**
 * rafl: ExampleReservoir.cpp
 */

#include "examples/ExampleReservoir.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <iterator>

namespace rafl {

//#################### PUBLIC MEMBER FUNCTIONS ####################

ExampleReservoir::ExampleReservoir(size_t maxSize, unsigned int seed)
: m_maxSize(maxSize), m_seenExamples(0)
{
  srand(seed);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ExampleReservoir::add_example(int exampleID)
{
  if(m_seenExamples < m_maxSize)
  {
    m_exampleIDs.push_back(exampleID);
  }
  else
  {
    size_t k = rand() % m_seenExamples;
    if(k < m_maxSize) m_exampleIDs[k] = exampleID;
  }

  ++m_seenExamples;
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const ExampleReservoir& rhs)
{
  std::copy(rhs.m_exampleIDs.begin(), rhs.m_exampleIDs.end(), std::ostream_iterator<int>(os, " "));
  return os;
}

}
