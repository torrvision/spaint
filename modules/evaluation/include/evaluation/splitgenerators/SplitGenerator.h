/**
 * evaluation: SplitGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_SPLITGENERATOR
#define H_EVALUATION_SPLITGENERATOR

#include <utility>
#include <vector>

#include <tvgutil/numbers/RandomNumberGenerator.h>

namespace evaluation {

/**
 * \brief An instance of a class deriving from this one can be used to generate example splits for learner evaluation.
 */
class SplitGenerator
{
  //#################### TYPEDEFS ####################
public:
  typedef std::pair<std::vector<size_t>,std::vector<size_t> > Split;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The random number generator to use when generating splits. */
  tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a split generator.
   *
   * \param seed  The seed to use for the random number generator when generating splits.
   */
  explicit SplitGenerator(unsigned int seed)
  : m_rng(seed)
  {}

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the split generator.
   */
  virtual ~SplitGenerator() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates (binary) splits of a set of examples that can be used for learner evaluation.
   *
   * \param exampleCount  The overall number of examples.
   * \return              A set of splits, each of which represents a way in which the examples can be split into two pieces.
   */
  virtual std::vector<Split> generate_splits(size_t exampleCount) = 0;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<SplitGenerator> SplitGenerator_Ptr;

}

#endif
