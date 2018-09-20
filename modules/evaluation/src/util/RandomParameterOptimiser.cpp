/**
 * evaluation: RandomParameterOptimiser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "util/RandomParameterOptimiser.h"

#include <limits>

#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
using boost::assign::list_of;
using boost::spirit::hold_any;

namespace evaluation {

//#################### CONSTRUCTORS ####################

RandomParameterOptimiser::RandomParameterOptimiser(const CostFunction& costFunction, size_t epochCount, unsigned int seed)
: m_costFunction(costFunction), m_epochCount(epochCount), m_rng(seed)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

RandomParameterOptimiser& RandomParameterOptimiser::add_param(const std::string& param, const std::vector<hold_any>& values)
{
  m_paramValues.push_back(std::make_pair(param, values));
  return *this;
}

ParamSet RandomParameterOptimiser::optimise_for_parameters(float *bestCost) const
{
  std::vector<size_t> bestValueIndicesAllTime;
  float bestCostAllTime = std::numeric_limits<float>::max();

  // For each epoch:
  for(size_t i = 0; i < m_epochCount; ++i)
  {
    // Randomly generate an initial set of parameter value indices.
    std::vector<size_t> valueIndices = generate_random_value_indices();

    // Optimise the initial set of parameter value indices using coordinate descent.
    float configurationCost = compute_cost(valueIndices);

    // If the optimised cost is the best we've seen so far, update the best cost and best parameter value indices.
    if(configurationCost < bestCostAllTime)
    {
      bestCostAllTime = configurationCost;
      bestValueIndicesAllTime = valueIndices;
    }
  }

  // Return the best parameters found and (optionally) the corresponding cost.
  if(bestCost) *bestCost = bestCostAllTime;
  return make_param_set(bestValueIndicesAllTime);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

float RandomParameterOptimiser::compute_cost(const std::vector<size_t>& valueIndices) const
{
  return m_costFunction(make_param_set(valueIndices));
}

std::vector<size_t> RandomParameterOptimiser::generate_random_value_indices() const
{
  std::vector<size_t> valueIndices;
  for(size_t i = 0, paramCount = m_paramValues.size(); i < paramCount; ++i)
  {
    valueIndices.push_back(m_rng.generate_int_from_uniform(0, static_cast<int>(m_paramValues[i].second.size()) - 1));
  }
  return valueIndices;
}

ParamSet RandomParameterOptimiser::make_param_set(const std::vector<size_t>& valueIndices) const
{
  ParamSet paramSet;
  for(size_t i = 0; i < m_paramValues.size(); ++i)
  {
    std::string param = m_paramValues[i].first;
    std::string value = boost::lexical_cast<std::string>(m_paramValues[i].second[valueIndices[i]]);
    paramSet.insert(std::make_pair(param, value));
  }
  return paramSet;
}

}
