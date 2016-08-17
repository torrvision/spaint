/**
 * evaluation: CoordinateDescentParameterSetGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <iostream>

#include "paramsetgenerators/CoordinateDescentParameterSetGenerator.h"

#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
using boost::spirit::hold_any;

#include <tvgutil/misc/ArgUtil.h>
#include <tvgutil/containers/LimitedContainer.h>
using namespace tvgutil;

//#define DEBUG_COORDINATE_DESCENT

namespace evaluation {

//#################### CONSTRUCTORS ####################

CoordinateDescentParameterSetGenerator::CoordinateDescentParameterSetGenerator(unsigned int seed, size_t epochCount)
: m_epochCount(epochCount),
  m_rng(seed)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

CoordinateDescentParameterSetGenerator& CoordinateDescentParameterSetGenerator::add_param(const std::string& param, const std::vector<boost::spirit::hold_any>& values)
{
  ParameterSetGenerator::add_param(param, values);
  return *this;
}

ParamSet CoordinateDescentParameterSetGenerator::calculate_best_parameters() const
{
  float dummy(0.0f);
  return calculate_best_parameters(dummy);
}

ParamSet CoordinateDescentParameterSetGenerator::calculate_best_parameters(float& bestScore) const
{
  const size_t maxIterationCount = get_iteration_count();
  for(size_t i = 0; i < maxIterationCount; ++i)
  {
    ParamSet params = get_next_param_set();
    score_param_set_and_update_state(params, m_costFunction(params));
  }

  bestScore = get_best_score();

  return get_best_param_set();
}

std::vector<ParamSet> CoordinateDescentParameterSetGenerator::generate_param_sets() const
{
  std::vector<ParamSet> result;
  result.push_back(calculate_best_parameters());
  return result;
}

void CoordinateDescentParameterSetGenerator::initialise(const boost::function<float(const ParamSet&)>& costFunction)
{
  m_costFunction = costFunction;

  m_firstIteration = true;

  // Initialise the number of dimensions.
  m_dimCount = m_paramValues.size();

  const float largeScore = 1e10f;
  m_bestScoreAllTime = m_bestScore = largeScore;
  m_globalParamCount = 0;
  m_singlePointParamCount = 0;
  for(size_t i = 0; i < m_dimCount; ++i)
  {
    size_t size = m_paramValues[i].second.size();
    m_globalParamCount += size;
    if(size == 1) ++m_singlePointParamCount;

    // Randomly initialise the best parameter indices.
    m_currentParamIndices.push_back(m_rng.generate_int_from_uniform(0, size - 1));

    // Initialise the score associated with each parameter to a very large value.
    m_paramScores.push_back(std::vector<float>(size, largeScore));
  }

  // The first dimension to search.
  m_currentDimIndex = m_firstDimIndex = m_rng.generate_int_from_uniform(0, m_dimCount - 1);

  if(m_globalParamCount != m_singlePointParamCount)
  {
    // Skip parameters that assume only one value.
    while(m_paramScores[m_currentDimIndex].size() <= 1)
    {
      ++m_currentDimIndex %= m_dimCount;
      m_firstDimIndex = m_currentDimIndex;
    }
  }

  // Initialise the index of parameters along the first coordinate to seach to the first element.
  m_currentParamIndices[m_currentDimIndex] = 0;

  // The current and initial parameter indices are assumend to be the best.
  m_bestParamIndicesAllTime = m_previousBestParamIndices = m_bestParamIndices = m_currentParamIndices;

#ifdef DEBUG_COORDINATE_DESCENT
  std::cout << "End of initialisation: \n";
  std::cout << "Current parameter indices: " << make_limited_container(m_currentParamIndices, 50) << std::endl;
  std::cout << "Best parameter indices: " << make_limited_container(m_bestParamIndices, 50) << std::endl;
  std::cout << "Best parameter indices all time: " << make_limited_container(m_bestParamIndicesAllTime, 50) << std::endl;
#endif
}

void CoordinateDescentParameterSetGenerator::output_param_values(std::ostream& os) const
{
  for(size_t i = 0, size = m_paramValues.size(); i < size; ++i)
  {
    std::string paramName = m_paramValues[i].first;
    std::vector<boost::spirit::hold_any> paramValues = m_paramValues[i].second;
    std::string paramString = paramName + ": ";
    for(size_t j = 0, size = paramValues.size(); j < size; ++j)
    {
      paramString += boost::lexical_cast<std::string>(paramValues[j]);
      if(j < size - 1) paramString += ", ";
    }

    os << paramString << '\n';
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

ParamSet CoordinateDescentParameterSetGenerator::get_best_param_set() const
{
  return param_indices_to_set(m_bestParamIndicesAllTime);
}


float CoordinateDescentParameterSetGenerator::get_best_score() const
{
  return m_bestScoreAllTime;
}

size_t CoordinateDescentParameterSetGenerator::get_iteration_count() const
{
  if(m_globalParamCount == m_singlePointParamCount)
  {
    return 1;
  }
  else
  {
    // If parameters only have a single value, then do not count the parameter in the iteration count.
    size_t epochIterationCount = m_globalParamCount - m_singlePointParamCount;
    return m_epochCount * epochIterationCount;
  }
}

ParamSet CoordinateDescentParameterSetGenerator::get_next_param_set() const
{
  m_currentParamSet = param_indices_to_set(m_currentParamIndices);
  return m_currentParamSet;
}

ParamSet CoordinateDescentParameterSetGenerator::param_indices_to_set(const std::vector<size_t>& paramIndices) const
{
  ParamSet paramSet;
  for(size_t i = 0; i < m_dimCount; ++i)
  {
    paramSet.insert(std::make_pair(m_paramValues[i].first, boost::lexical_cast<std::string>(m_paramValues[i].second[paramIndices[i]])));
  }

  return paramSet;
}

void CoordinateDescentParameterSetGenerator::random_restart() const
{
  m_currentDimIndex = m_firstDimIndex = m_rng.generate_int_from_uniform(0, m_dimCount - 1);
  for(size_t i = 0; i < m_dimCount; ++i)
  {
    size_t size = m_paramValues[i].second.size();
    m_currentParamIndices[i] = m_rng.generate_int_from_uniform(0, size - 1);
  }
}

void CoordinateDescentParameterSetGenerator::score_param_set_and_update_state(const ParamSet& paramSet, float score) const
{
  if(paramSet == m_currentParamSet)
  {
    m_paramScores[m_currentDimIndex][m_currentParamIndices[m_currentDimIndex]] = score;
    update_state();
  }
  else
  {
    std::cout << "Warning: the parameter set for which you are entering a score does not match the current parameter set.\n";
  }
}

void CoordinateDescentParameterSetGenerator::update_state() const
{
  if(m_currentParamIndices[m_currentDimIndex] < m_paramScores[m_currentDimIndex].size() -1)
  {
    ++m_currentParamIndices[m_currentDimIndex];
  }
  else
  {
    size_t bestIndex = ArgUtil::argmin(m_paramScores[m_currentDimIndex]);
    m_bestScore = m_paramScores[m_currentDimIndex][bestIndex];

#ifdef DEBUG_COORDINATE_DESCENT
    std::cout << "Finished going through the values along dimension:" << m_currentDimIndex;
    std::cout << ":" << m_paramValues[m_currentDimIndex].first << '\n';
    std::cout << "bestIndex:" << bestIndex << '\n';
    std::cout << "scoreAtBestIndex:" << m_bestScore << '\n';
#endif

    m_currentParamIndices[m_currentDimIndex] = m_bestParamIndices[m_currentDimIndex] = bestIndex;

    if(m_bestScore < m_bestScoreAllTime)
    {
      m_bestScoreAllTime = m_bestScore;
      m_bestParamIndicesAllTime = m_bestParamIndices;
    }

    // Make sure the current dimension index does not go out of bounds when incremented.
    ++m_currentDimIndex %= m_dimCount;

    // If we have done a full circle and stil have the same best parameters..
    if(m_currentDimIndex == m_firstDimIndex)
    {
      if(m_bestParamIndices == m_previousBestParamIndices)
      {
#ifdef DEBUG_COORDINATE_DESCENT
        std::cout << "<<<< Converged >>>>\n";
        std::cout << "Going for a random restart\n";
#endif
        random_restart();
      }

#ifdef DEBUG_COORDINATE_DESCENT
      std::cout << "**** Going for another pass ****\n";
#endif
      m_previousBestParamIndices = m_bestParamIndices;
    }

    if(m_globalParamCount != m_singlePointParamCount)
    {
      // Skip parameters that assume only one value.
      while(m_paramScores[m_currentDimIndex].size() <= 1)
      {
        ++m_currentDimIndex %= m_dimCount;
      }
    }

    m_currentParamIndices[m_currentDimIndex] = 0;
  }


#ifdef DEBUG_COORDINATE_DESCENT
  std::cout << "After update state: \n";
  std::cout << "Current parameter indices: " << make_limited_container(m_currentParamIndices, 50) << std::endl;
  std::cout << "Best parameter indices: " << make_limited_container(m_bestParamIndices, 50) << std::endl;
  std::cout << "Best parameter indices all time: " << make_limited_container(m_bestParamIndicesAllTime, 50) << std::endl;
#endif

}

}
