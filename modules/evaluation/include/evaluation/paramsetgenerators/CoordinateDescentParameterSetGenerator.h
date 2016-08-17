/**
 * evaluation: CoordinateDescentParameterSetGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_EVALUATION_COORDINATEDESCENTPARAMETERSETGENERATOR
#define H_EVALUATION_COORDINATEDESCENTPARAMETERSETGENERATOR

#include <boost/function.hpp>

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
  /** The cost function. */
  boost::function<float(const ParamSet&)> m_costFunction;

  /** The number of passes that should be made through the parameter set. */
  size_t m_epochCount;

  /** The random number generator to use when deciding which parameters to search first. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  CoordinateDescentParameterSetGenerator(unsigned int seed, size_t epochCount, const boost::function<float(const ParamSet&)>& costFunction);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  ParamSet calculate_best_parameters(float *bestCost = NULL) const;

  /** Override */
  virtual std::vector<ParamSet> generate_param_sets() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  float compute_cost(const std::vector<size_t>& valueIndices) const;

  /**
   * \brief TODO
   */
  std::vector<size_t> generate_random_value_indices() const;

  /**
   * \brief Converts a set of parameter value indices to a parameter set.
   *
   * \param valueIndices  The index set.
   * \return              The parameter set corresponding to the index set.
   */
  ParamSet make_param_set(const std::vector<size_t>& valueIndices) const;

  /**
   * \brief TODO
   */
  std::pair<std::vector<size_t>,float> optimise(const std::vector<size_t>& initialValueIndices) const;
};

}

#endif
