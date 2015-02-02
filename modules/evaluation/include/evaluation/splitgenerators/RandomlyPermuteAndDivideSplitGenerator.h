/**
 * evaluation: RandomlyPermuteAndDivideSplitGenerator.h
 */

#ifndef H_EVALUATION_RANDOMLYPERMUTEANDDIVIDESPLITGENERATOR
#define H_EVALUATION_RANDOMLYPERMUTEANDDIVIDESPLITGENERATOR

#include <tvgutil/RandomNumberGenerator.h>

#include "SplitGenerator.h"

namespace evaluation {

/**
 * \brief TODO
 */
class RandomlyPermuteAndDivideSplitGenerator : public SplitGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of folds to use. */
  size_t m_foldCount;

  /** The ratio [number of examples in the training set]/[number of examples in the validation set]. */
  float m_ratio;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   *
   * \param foldCount The number of folds to use.
   * \param seed      The seed to use for the random number generator when generating splits.
   * \param ratio     The ratio [number of examples in the training set]/[number of examples in the validation set].
   */
  RandomlyPermuteAndDivideSplitGenerator(unsigned int seed, size_t foldCount, float ratio);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::vector<Split> generate_splits(size_t exampleCount);

};

}

#endif
