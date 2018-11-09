/**
 * evaluation: RandomParameterOptimiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_EVALUATION_RANDOMPARAMETEROPTIMISER
#define H_EVALUATION_RANDOMPARAMETEROPTIMISER

#include "EpochBasedParameterOptimiser.h"

namespace evaluation {

/**
 * \brief An instance of this class uses random parameter generation to find a parameter set with as low a cost as possible.
 */
class RandomParameterOptimiser : public EpochBasedParameterOptimiser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random parameter optimiser.
   *
   * \param costFunction  The cost function to use to evaluate the different parameter sets.
   * \param epochCount    The number of epochs for which the random parameter generation should be run.
   * \param seed          The seed for the random number generator.
   */
  RandomParameterOptimiser(const CostFunction& costFunction, size_t epochCount, unsigned int seed);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual std::pair<std::vector<size_t>,float> optimise_value_indices(const std::vector<size_t>& initialValueIndices) const;
};

}

#endif
