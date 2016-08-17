/**
 * evaluation: CoordinateDescentParameterSetGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <iostream>
#include <limits>

#include "paramsetgenerators/CoordinateDescentParameterSetGenerator.h"

#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
using boost::assign::list_of;
using boost::spirit::hold_any;

#include <tvgutil/misc/ArgUtil.h>
#include <tvgutil/containers/LimitedContainer.h>
using namespace tvgutil;

//#define DEBUG_COORDINATE_DESCENT

namespace evaluation {

//#################### CONSTRUCTORS ####################

CoordinateDescentParameterSetGenerator::CoordinateDescentParameterSetGenerator(unsigned int seed, size_t epochCount, const boost::function<float(const ParamSet&)>& costFunction)
: /*m_bestScore(std::numeric_limits<float>::max()),
  m_bestScoreAllTime(std::numeric_limits<float>::max()),*/
  m_costFunction(costFunction),
  m_epochCount(epochCount),
  m_rng(seed)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

#if 0
ParamSet CoordinateDescentParameterSetGenerator::calculate_best_parameters2(float *bestScore) const
{
  const size_t maxIterationCount = get_iteration_count();
  for(size_t i = 0; i < maxIterationCount; ++i)
  {
    const ParamSet paramSet = param_indices_to_set(m_currentParamIndices);
    m_paramScores[m_currentDimIndex][m_currentParamIndices[m_currentDimIndex]] = m_costFunction(paramSet);
    update_state();
  }

  if(bestScore) *bestScore = m_bestScoreAllTime;

  return param_indices_to_set(m_bestParamIndicesAllTime);
}
#endif

ParamSet CoordinateDescentParameterSetGenerator::calculate_best_parameters(float *bestScore) const
{
  std::vector<size_t> bestParamIndicesAllTime;
  float bestScoreAllTime = static_cast<float>(INT_MAX);

  for(size_t epochIndex = 0; epochIndex < m_epochCount; ++epochIndex)
  {
    std::vector<size_t> bestParamIndicesInEpoch;
    float bestScoreInEpoch;
    boost::tie(bestParamIndicesInEpoch, bestScoreInEpoch) = run_epoch();

    if(bestScoreInEpoch < bestScoreAllTime)
    {
      bestScoreAllTime = bestScoreInEpoch;
      bestParamIndicesAllTime = bestParamIndicesInEpoch;
    }
  }

  if(bestScore) *bestScore = bestScoreAllTime;

  return param_indices_to_set(bestParamIndicesAllTime);
}

std::pair<std::vector<size_t>,float> CoordinateDescentParameterSetGenerator::run_epoch() const
{
  std::vector<size_t> currentParamIndices = generate_random_param_indices();
  float currentScore = m_costFunction(param_indices_to_set(currentParamIndices));

  std::vector<size_t> bestParamIndicesInEpoch = currentParamIndices;
  float bestScoreInEpoch = currentScore;

  size_t paramCount = m_paramValues.size();
  int startingOffset = m_rng.generate_int_from_uniform(0, static_cast<int>(paramCount) - 1);
  for(size_t k = 0; k < paramCount; ++k)
  {
    size_t paramIndex = (k + startingOffset) % paramCount;

    int valueCount = static_cast<int>(m_paramValues[paramIndex].second.size());
    if(valueCount == 1) continue;

    size_t originalValueIndex = currentParamIndices[paramIndex];
    std::vector<size_t> valueIndices = currentParamIndices;

    for(size_t valueIndex = 0; valueIndex < valueCount; ++valueIndex)
    {
      if(valueIndex == originalValueIndex) continue;

      valueIndices[paramIndex] = valueIndex;
      float score = m_costFunction(param_indices_to_set(valueIndices));

      if(score < currentScore)
      {
        currentParamIndices[paramIndex] = valueIndex;
        currentScore = score;
      }
    }

    if(currentScore < bestScoreInEpoch)
    {
      bestParamIndicesInEpoch = currentParamIndices;
      bestScoreInEpoch = currentScore;
    }
  }

  return std::make_pair(bestParamIndicesInEpoch, bestScoreInEpoch);
}

std::vector<size_t> CoordinateDescentParameterSetGenerator::generate_random_param_indices() const
{
  std::vector<size_t> result;
  for(size_t i = 0, paramCount = m_paramValues.size(); i < paramCount; ++i)
  {
    result.push_back(m_rng.generate_int_from_uniform(0, static_cast<int>(m_paramValues[i].second.size()) - 1));
  }
  return result;
}

std::vector<ParamSet> CoordinateDescentParameterSetGenerator::generate_param_sets() const
{
  return list_of(calculate_best_parameters());
}

#if 0
void CoordinateDescentParameterSetGenerator::initialise()
{
  m_globalParamCount = 0;
  m_singlePointParamCount = 0;
  for(size_t i = 0; i < m_paramValues.size(); ++i)
  {
    const size_t numValuesForParam = m_paramValues[i].second.size();
    m_globalParamCount += numValuesForParam;
    if(numValuesForParam == 1) ++m_singlePointParamCount;

    // Pick a random initial value for this parameter from the available options and store its index.
    m_currentParamIndices.push_back(m_rng.generate_int_from_uniform(0, numValuesForParam - 1));

    // Initialise the score associated with each parameter to a very large value.
    m_paramScores.push_back(std::vector<float>(numValuesForParam, std::numeric_limits<float>::max()));
  }

  // The first dimension to search.
  m_currentDimIndex = m_firstDimIndex = m_rng.generate_int_from_uniform(0, m_paramValues.size() - 1);

  if(m_globalParamCount != m_singlePointParamCount)
  {
    // Skip parameters that assume only one value.
    while(m_paramScores[m_currentDimIndex].size() <= 1)
    {
      ++m_currentDimIndex %= m_paramValues.size();
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

//#################### PRIVATE MEMBER FUNCTIONS ####################

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
#endif

ParamSet CoordinateDescentParameterSetGenerator::param_indices_to_set(const std::vector<size_t>& paramIndices) const
{
  ParamSet paramSet;
  for(size_t i = 0; i < m_paramValues.size(); ++i)
  {
    paramSet.insert(std::make_pair(m_paramValues[i].first, boost::lexical_cast<std::string>(m_paramValues[i].second[paramIndices[i]])));
  }

  return paramSet;
}

#if 0
void CoordinateDescentParameterSetGenerator::random_restart() const
{
  m_currentDimIndex = m_firstDimIndex = m_rng.generate_int_from_uniform(0, m_paramValues.size() - 1);
  for(size_t i = 0; i < m_paramValues.size(); ++i)
  {
    size_t size = m_paramValues[i].second.size();
    m_currentParamIndices[i] = m_rng.generate_int_from_uniform(0, size - 1);
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
    ++m_currentDimIndex %= m_paramValues.size();

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
        ++m_currentDimIndex %= m_paramValues.size();
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
#endif

}
