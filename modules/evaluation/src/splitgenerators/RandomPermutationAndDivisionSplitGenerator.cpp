/**
 * evaluation: RandomPermutationAndDivisionSplitGenerator.cpp
 */

#include "splitgenerators/RandomPermutationAndDivisionSplitGenerator.h"
#include "splitgenerators/RNGFunctor.h"

#include <cassert>
#include <stdexcept>

#include <tvgutil/LimitedContainer.h>

namespace evaluation {

//#################### CONSTRUCTORS ####################

RandomPermutationAndDivisionSplitGenerator::RandomPermutationAndDivisionSplitGenerator(unsigned int seed, size_t splitCount, float ratio)
: SplitGenerator(seed), m_ratio(ratio), m_splitCount(splitCount)
{
  assert(m_splitCount > 1);
  assert(m_ratio > 0.0f && m_ratio < 1.0f);

  if(m_ratio < 0.1f || m_ratio > 0.9f)
  {
    std::cout << "Warning: the ratio value being used: " << m_ratio << " seems unwise.\n";
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::vector<SplitGenerator::Split> RandomPermutationAndDivisionSplitGenerator::generate_splits(size_t exampleCount)
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
  for(size_t i = 0; i < m_splitCount; ++i)
  {
    Split split;

    // Randomly shuffle the indices.
    std::random_shuffle(exampleIndices.begin(), exampleIndices.end(), RNGFunctor(m_rng));
    split.first.insert(split.first.begin(), exampleIndices.begin(), exampleIndices.begin() + firstSetSize);
    split.second.insert(split.second.begin(), exampleIndices.begin() + firstSetSize + 1, exampleIndices.end());

#if 1
    std::cout << "First: \n" << tvgutil::make_limited_container(split.first, 20) << '\n';
    std::cout << "Second: \n" << tvgutil::make_limited_container(split.second, 20) << "\n\n";
#endif

    splits.push_back(split);
  }

  return splits;
}

}
