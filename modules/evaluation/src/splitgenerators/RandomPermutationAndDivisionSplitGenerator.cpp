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
  if(m_splitCount == 0)
  {
    throw std::runtime_error("Must generate at least one split");
  }

  if(m_ratio <= 0.0f || m_ratio >= 1.0f)
  {
    throw std::runtime_error("Split ratio out of range");
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

  std::vector<Split> splits;
  size_t trainingSetSize = static_cast<size_t>(m_ratio * exampleCount);
  for(size_t i = 0; i < m_splitCount; ++i)
  {
    // Randomly shuffle the indices and construct the split.
    Split split;
    std::random_shuffle(exampleIndices.begin(), exampleIndices.end(), RNGFunctor(m_rng));
    std::vector<size_t>& trainingSet = split.first;
    std::vector<size_t>& validationSet = split.second;
    split.first.insert(trainingSet.begin(), exampleIndices.begin(), exampleIndices.begin() + trainingSetSize);
    split.second.insert(validationSet.begin(), exampleIndices.begin() + trainingSetSize, exampleIndices.end());

#if 0
    std::cout << "Training: \n" << tvgutil::make_limited_container(trainingSet, 20) << '\n';
    std::cout << "Validation: \n" << tvgutil::make_limited_container(validationSet, 20) << "\n\n";
#endif

    splits.push_back(split);
  }

  return splits;
}

}
