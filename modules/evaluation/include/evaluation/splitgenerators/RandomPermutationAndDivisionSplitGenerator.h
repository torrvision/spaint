/**
 * evaluation: RandomPermutationAndDivisionSplitGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_RANDOMPERMUTATIONANDDIVISIONSPLITGENERATOR
#define H_EVALUATION_RANDOMPERMUTATIONANDDIVISIONSPLITGENERATOR

#include <tvgutil/numbers/RandomNumberGenerator.h>

#include "SplitGenerator.h"

namespace evaluation {

/**
 * \brief An instance of this class can be used to generate example splits for learner evaluation.
 *
 * The approach used here first randomly permutes the example indices, then divides them into two sets,
 * the first of which contains a specified percentage of the examples, and the second of which contains
 * the remaining examples. This is repeated for each of the specified number of splits.
 */
class RandomPermutationAndDivisionSplitGenerator : public SplitGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A ratio, namely [the number of examples to assign to the training set for each split] / [the total number of examples]. */
  float m_ratio;

  /** The number of splits to generate. */
  size_t m_splitCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a "random permutation and division" split generator.
   *
   * Note that the ratio must be strictly between 0 and 1.
   *
   * \param seed        The seed to use for the random number generator when generating splits.
   * \param splitCount  The number of splits to generate.
   * \param ratio       A ratio, namely [the number of examples to assign to the training set for each split] / [the total number of examples].
   */
  RandomPermutationAndDivisionSplitGenerator(unsigned int seed, size_t splitCount, float ratio);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::vector<Split> generate_splits(size_t exampleCount);

};

}

#endif
