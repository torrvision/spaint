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
 * \brief An instance of this class uses coordinate descent with random restarts to find a parameter set with as low a cost as possible.
 */
class CoordinateDescentParameterSetGenerator : public ParameterSetGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The cost function to use to evaluate different parameter sets. */
  boost::function<float(const ParamSet&)> m_costFunction;

  /** The number of epochs for which coordinate descent should be run. */
  size_t m_epochCount;

  /** A random number generator. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a coordinate descent parameter set generator.
   *
   * \param costFunction  The cost function to use to evaluate different parameter sets.
   * \param epochCount    The number of epochs for which coordinate descent should be run.
   * \param seed          The seed for the random number generator.
   */
  CoordinateDescentParameterSetGenerator(const boost::function<float(const ParamSet&)>& costFunction, size_t epochCount, unsigned int seed);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Performs coordinate descent optimisation with random restarts to try to find a parameter set with as low a cost as possible.
   *
   * \param bestCost  A place in which to return the cost associated with the best parameter set found (may be NULL).
   * \return          The best parameter set found during coordinate descent.
   */
  ParamSet choose_parameters(float *bestCost = NULL) const;

  /** Override */
  virtual std::vector<ParamSet> generate_param_sets() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes the cost associated with setting the parameters being optimised to the values denoted by the
   *        specified parameter value indices.
   *
   * \param valueIndices  A set of parameter value indices, denoting particular settings for the parameters.
   * \return              The cost associated with setting the parameters being optimised to the denoted values.
   */
  float compute_cost(const std::vector<size_t>& valueIndices) const;

  /**
   * \brief Generates a random set of parameter value indices, denoting particular settings for the parameters.
   *
   * \return  A random set of parameter value indices.
   */
  std::vector<size_t> generate_random_value_indices() const;

  /**
   * \brief Makes the parameter set corresponding to the specified parameter value indices.
   *
   * \param valueIndices  A set of parameter value indices, denoting particular settings for the parameters.
   * \return              The corresponding parameter set.
   */
  ParamSet make_param_set(const std::vector<size_t>& valueIndices) const;

  /**
   * \brief Performs coordinate descent optimisation to optimise an initial set of parameter value indices
   *        in order to minimise the associated cost.
   *
   * \param initialValueIndices The initial set of parameter value indices.
   * \return                    An optimised set of parameter value indices and the associated cost.
   */
  std::pair<std::vector<size_t>,float> optimise(const std::vector<size_t>& initialValueIndices) const;
};

}

#endif
