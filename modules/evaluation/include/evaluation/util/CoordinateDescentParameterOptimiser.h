/**
 * evaluation: CoordinateDescentParameterOptimiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_EVALUATION_COORDINATEDESCENTPARAMETEROPTIMISER
#define H_EVALUATION_COORDINATEDESCENTPARAMETEROPTIMISER

#include "EpochBasedParameterOptimiser.h"

namespace evaluation {

/**
 * \brief An instance of this class uses coordinate descent with random restarts to find a parameter set with as low a cost as possible.
 */
class CoordinateDescentParameterOptimiser : public EpochBasedParameterOptimiser
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a coordinate descent parameter optimiser.
   *
   * \param costFunction  The cost function to use to evaluate the different parameter sets.
   * \param epochCount    The number of epochs for which coordinate descent should be run.
   * \param seed          The seed for the random number generator.
   */
  CoordinateDescentParameterOptimiser(const CostFunction& costFunction, size_t epochCount, unsigned int seed);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual std::pair<std::vector<size_t>,float> optimise_value_indices(const std::vector<size_t>& initialValueIndices) const;
};

}

#endif
