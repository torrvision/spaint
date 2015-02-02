/**
 * evaluation: CrossValidationSplitGenerator.h
 */

#ifndef H_EVALUATION_CROSSVALIDATIONSPLITGENERATOR
#define H_EVALUATION_CROSSVALIDATIONSPLITGENERATOR

#include <tvgutil/RandomNumberGenerator.h>

#include "SplitGenerator.h"

namespace evaluation {

/**
 * \brief An instance of this class can be used to generate example splits for cross-validation.
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
   * \param foldCount The number of folds to use for the cross-validation.
   * \param seed      The seed to use for the random number generator when generating splits.
   */
  CrossValidationSplitGenerator(unsigned int seed, size_t foldCount);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::vector<Split> generate_splits(size_t exampleCount);

};

}

#endif
