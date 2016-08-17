/**
 * evaluation: CoordinateDescentParameterSetGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_EVALUATION_COORDINATEDESCENTPARAMETERSETGENERATOR
#define H_EVALUATION_COORDINATEDESCENTPARAMETERSETGENERATOR

#include <string>
#include <utility>
#include <vector>

#include <boost/function.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <tvgutil/numbers/RandomNumberGenerator.h>

#include "ParameterSetGenerator.h"

namespace evaluation {

/**
 * \brief An instance of this class will try to find the parameters that minimise a function using coordinate descent.
 */
class CoordinateDescentParameterSetGenerator : public ParameterSetGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The best parameter indices along each parameter dimension. */
  mutable std::vector<size_t> m_bestParamIndices;

  /** The best parameter indices over all epochs. */
  mutable std::vector<size_t> m_bestParamIndicesAllTime;

  /** The best score within the current epoch. */
  mutable float m_bestScore;

  /** The best score over all epochs. */
  mutable float m_bestScoreAllTime;

  /** The cost function. */
  boost::function<float(const ParamSet&)> m_costFunction;

  /** The dimension index along which to search. */
  mutable size_t m_currentDimIndex;

  /** The current parameter indices for which a score is needed. */
  mutable std::vector<size_t> m_currentParamIndices;

  /** The total number of parameter dimensions. */
  size_t m_dimCount;

  /** The number of passes that should be made through the parameter set. */
  size_t m_epochCount;

  /** The first dimension along which to search. */
  mutable size_t m_firstDimIndex;

  /** A flag indicating whether the parameter generator has passed the first iteration. */
  mutable bool m_firstIteration;

  /** The total number of parameters that need to be searched in one epoch. */
  size_t m_globalParamCount;

  /** A list of the scores associated with each parameter. */
  mutable std::vector<std::vector<float> > m_paramScores;

  /** A record of the best parameter indices obtained in the last epoch. */
  mutable std::vector<size_t> m_previousBestParamIndices;

  /** The random number generator to use when deciding which parameters to search first. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  /** The total number of independent parameters that have only one value. */
  size_t m_singlePointParamCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  CoordinateDescentParameterSetGenerator(unsigned int seed, size_t epochCount, const boost::function<float(const ParamSet&)>& costFunction);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculate the best parameters.
   *
   * \param bestScore  Sets the best score associated with the best parameter set.
   *
   * \return The best parameters.
   */
  ParamSet calculate_best_parameters(float *bestScore = NULL) const;

  /** Override */
  virtual std::vector<ParamSet> generate_param_sets() const;

  /** Override */
  virtual void initialise();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the total number of iterations.
   *
   * \return The total number of iterations.
   */
  size_t get_iteration_count() const;

  /**
   * \brief Converts a set of parameter indices to a parameter set.
   *
   * \param paramIndices  The index set.
   * \return              The parameter set corresponding to the index set.
   */
  ParamSet param_indices_to_set(const std::vector<size_t>& paramIndices) const;

  /**
   * \brief Randomly restart the parameter search when learning has converged.
   */
  void random_restart() const;

  /**
   * \brief Update the current state of the parameter generator.
   */
  void update_state() const;
};

}

#endif
