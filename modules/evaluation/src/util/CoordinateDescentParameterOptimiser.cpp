/**
 * evaluation: CoordinateDescentParameterOptimiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "util/CoordinateDescentParameterOptimiser.h"

namespace evaluation {

//#################### CONSTRUCTORS ####################

CoordinateDescentParameterOptimiser::CoordinateDescentParameterOptimiser(const CostFunction& costFunction, size_t epochCount, unsigned int seed)
: EpochBasedParameterOptimiser(costFunction, epochCount, seed)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

std::pair<std::vector<size_t>,float> CoordinateDescentParameterOptimiser::optimise_value_indices(const std::vector<size_t>& initialValueIndices) const
{
  // Invariant: currentCost = compute_cost(currentValueIndices)

  // Initialise the current and best parameter value indices and their associated costs.
  std::vector<size_t> bestValueIndices, currentValueIndices;
  float bestCost, currentCost;
  bestValueIndices = currentValueIndices = initialValueIndices;
  bestCost = currentCost = compute_cost(currentValueIndices);

  // Pick the first parameter to optimise. The parameters will be optimised one-by-one, starting from this parameter.
  size_t paramCount = m_paramValues.size();
  int startingParamIndex = m_rng.generate_int_from_uniform(0, static_cast<int>(paramCount) - 1);

  // For each parameter, starting from the one just chosen:
  for(size_t k = 0; k < paramCount; ++k)
  {
    size_t paramIndex = (startingParamIndex + k) % paramCount;

    // Check how many possible values the parameter can take. If there's only one possibility, the parameter can be skipped.
    size_t valueCount = m_paramValues[paramIndex].second.size();
    if(valueCount == 1) continue;

    // Record the parameter value for which we already have the corresponding cost so that we can avoid re-evaluating it.
    size_t originalValueIndex = currentValueIndices[paramIndex];

    // For each possible new value that the parameter can take:
    std::vector<size_t> newValueIndices = currentValueIndices;
    for(size_t valueIndex = 0; valueIndex < valueCount; ++valueIndex)
    {
      // If we already know that the cost for the new value is no better than the cost for the current value, skip it.
      if(valueIndex == originalValueIndex) continue;

      // Compute the cost for the new value. If it's better than the cost for the current value, update the current value.
      newValueIndices[paramIndex] = valueIndex;
      float newCost = compute_cost(newValueIndices);
      if(newCost < currentCost)
      {
        currentValueIndices[paramIndex] = valueIndex;
        currentCost = newCost;
      }
    }

    // If the current cost is now the best, update the best parameter value indices and the associated cost.
    if(currentCost < bestCost)
    {
      bestValueIndices = currentValueIndices;
      bestCost = currentCost;
    }
  }

  return std::make_pair(bestValueIndices, bestCost);
}

}
