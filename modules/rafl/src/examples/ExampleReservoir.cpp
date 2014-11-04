/**
 * rafl: ExampleReservoir.cpp
 */

#include "examples/ExampleReservoir.h"

#include <algorithm>
#include <iostream>
#include <iterator>

namespace rafl {

//#################### PUBLIC MEMBER FUNCTIONS ####################

ExampleReservoir::ExampleReservoir(size_t maxSize, unsigned int seed)
: m_maxSize(maxSize), m_seenExamples(0)
{
  m_gen.reset(new std::default_random_engine(seed));
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
    std::uniform_int_distribution<> dist(0, static_cast<int>(m_seenExamples) - 1);
    size_t k = dist(*m_gen);
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
