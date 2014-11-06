/**
 * rafl: ExampleReservoir.cpp
 */

#include "examples/ExampleReservoir.h"
using namespace tvgutil;

#include <algorithm>
#include <iostream>
#include <iterator>

namespace rafl {

//#################### PUBLIC MEMBER FUNCTIONS ####################

ExampleReservoir::ExampleReservoir(size_t maxSize, const RandomNumberGenerator_Ptr& rng)
: m_maxSize(maxSize), m_rng(rng), m_seenExamples(0)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ExampleReservoir::add_example(int exampleID)
{
  if(m_seenExamples < m_maxSize)
  {
    m_exampleIDs.push_back(exampleID);
  }
  else
  {
    size_t k = m_rng->generate_int_in_range(0, static_cast<int>(m_seenExamples) - 1);
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
