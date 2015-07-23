/**
 * evaluation: CrossValidationSplitGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_CROSSVALIDATIONSPLITGENERATOR
#define H_EVALUATION_CROSSVALIDATIONSPLITGENERATOR

#include "SplitGenerator.h"

namespace evaluation {

/**
 * \brief An instance of this class can be used to generate example splits for learner evaluation.
 *
 * The approach used here is cross-validation, which splits the examples into a number of "folds"
 * and generates a split for each fold, in which the split's fold is used as a validation set and
 * the remaining folds are used as training data.
 */
class CrossValidationSplitGenerator : public SplitGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of folds to use for the cross-validation. */
  size_t m_foldCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a cross-validation split generator.
   *
   * \param seed      The seed to use for the random number generator when generating splits.
   * \param foldCount The number of folds to use for the cross-validation.
   */
  CrossValidationSplitGenerator(unsigned int seed, size_t foldCount);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::vector<Split> generate_splits(size_t exampleCount);

};

}

#endif
