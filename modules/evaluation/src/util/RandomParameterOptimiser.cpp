/**
 * evaluation: RandomParameterOptimiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "util/RandomParameterOptimiser.h"

namespace evaluation {

//#################### CONSTRUCTORS ####################

RandomParameterOptimiser::RandomParameterOptimiser(const CostFunction& costFunction, size_t epochCount, unsigned int seed)
: EpochBasedParameterOptimiser(costFunction, epochCount, seed)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

std::pair<std::vector<size_t>,float> RandomParameterOptimiser::optimise_value_indices(const std::vector<size_t>& initialValueIndices) const
{
  // Don't perform any actual optimisation, just compute the cost of the initial parameters.
  return std::make_pair(initialValueIndices, compute_cost(initialValueIndices));
}

}
