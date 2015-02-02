/**
 * evaluation: RandomlyPermuteAndDivideSplitGenerator.cpp
 */

#include "splitgenerators/RandomlyPermuteAndDivideSplitGenerator.h"

#include <cassert>
#include <stdexcept>

#include <tvgutil/LimitedContainer.h>

namespace evaluation {

//#################### CONSTRUCTORS ####################

RandomlyPermuteAndDivideSplitGenerator::RandomlyPermuteAndDivideSplitGenerator(unsigned int seed, size_t foldCount, float ratio)
: SplitGenerator(seed), m_foldCount(foldCount), m_ratio(ratio)
{
  assert(m_foldCount > 1);
  assert(m_ratio > 0.0f && m_ratio < 1.0f);

  if(m_ratio < 0.1f || m_ratio > 0.9f)
  {
    std::cout << "Warning: the ratio value being used: " << m_ratio << " seems unwise.\n";
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::vector<SplitGenerator::Split> RandomlyPermuteAndDivideSplitGenerator::generate_splits(size_t exampleCount)
{
  if(exampleCount <= 2)
  {
    throw std::runtime_error("Too few examples available to generate splits");
  }

  std::vector<size_t> exampleIndices(exampleCount);
  for(size_t i = 0; i < exampleCount; ++i)
  {
    exampleIndices[i] = i;
  }

#if 1
  std::cout << "exampleIndices: \n" << tvgutil::make_limited_container(exampleIndices, 20) << '\n';
#endif

  size_t firstSetSize = static_cast<size_t>(m_ratio * exampleCount);
  std::vector<Split> splits;
  for(size_t fold = 0; fold < m_foldCount; ++fold)
  {
    Split splitSet;

    // Randomly shuffle the indices.
    std::random_shuffle(exampleIndices.begin(), exampleIndices.end());
    splitSet.first.insert(splitSet.first.begin(), exampleIndices.begin(), exampleIndices.begin() + firstSetSize);
    splitSet.second.insert(splitSet.second.begin(), exampleIndices.begin() + firstSetSize + 1, exampleIndices.end());

#if 1
    std::cout << "First: \n" << tvgutil::make_limited_container(splitSet.first, 20) << "\n";
    std::cout << "Second: \n" << tvgutil::make_limited_container(splitSet.second, 20) << "\n\n";
#endif

    splits.push_back(splitSet);
  }

  return splits;
}

}
