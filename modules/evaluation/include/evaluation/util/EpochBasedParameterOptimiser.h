/**
 * evaluation: EpochBasedParameterOptimiser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_EVALUATION_EPOCHBASEDPARAMETEROPTIMISER
#define H_EVALUATION_EPOCHBASEDPARAMETEROPTIMISER

#include <boost/function.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <tvgutil/numbers/RandomNumberGenerator.h>

#include "../core/ParamSetUtil.h"

namespace evaluation {

/**
 * \brief An instance of a class deriving from this one can be used to try to find (over a series of optimisation epochs) a parameter set with as low a cost as possible.
 */
class EpochBasedParameterOptimiser
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::function<float(const ParamSet&)> CostFunction;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The cost function to use to evaluate the different parameter sets. */
  CostFunction m_costFunction;

  /** The number of epochs for which optimisation should be run. */
  size_t m_epochCount;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** A list of the possible values for each parameter (e.g. [("A", [1,2]), ("B", [3,4])]). */
  std::vector<std::pair<std::string,std::vector<boost::spirit::hold_any> > > m_paramValues;

  /** A random number generator. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an epoch-based parameter optimiser.
   *
   * \param costFunction  The cost function to use to evaluate the different parameter sets.
   * \param epochCount    The number of epochs for which coordinate descent should be run.
   * \param seed          The seed for the random number generator.
   */
  EpochBasedParameterOptimiser(const CostFunction& costFunction, size_t epochCount, unsigned int seed);

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Optimises an initial set of parameter value indices in order to minimise the associated cost.
   *
   * \param initialValueIndices The initial set of parameter value indices.
   * \return                    An optimised set of parameter value indices and the associated cost.
   */
  virtual std::pair<std::vector<size_t>,float> optimise_value_indices(const std::vector<size_t>& initialValueIndices) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a parameter, together with a list of the values it may take.
   *
   * \param param   The parameter name.
   * \param values  The values the parameter may take.
   * \return        The optimiser itself (so that calls to add_param may be chained).
   */
  EpochBasedParameterOptimiser& add_param(const std::string& param, const std::vector<boost::spirit::hold_any>& values);

  /**
   * \brief Adds a parameter, together with a list of the values it may take.
   *
   * \param param   The parameter name.
   * \param values  The values the parameter may take.
   * \return        The optimiser itself (so that calls to add_param may be chained).
   */
  template <typename T>
  EpochBasedParameterOptimiser& add_param(const std::string& param, const std::vector<T>& values)
  {
    std::vector<boost::spirit::hold_any> anyValues(values.begin(), values.end());
    return add_param(param, anyValues);
  }

  /**
   * \brief Performs an epoch-based optimisation with random restarts to try to find a parameter set with as low a cost as possible.
   *
   * \param bestCost    A place in which to return the cost associated with the best parameter set found (may be NULL).
   * \return            The best parameter set found during the optimisation.
   */
  ParamSet optimise_for_parameters(float *bestCost = NULL) const;

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Computes the cost associated with setting the parameters being optimised to the values denoted by the
   *        specified parameter value indices.
   *
   * \param valueIndices  A set of parameter value indices, denoting particular settings for the parameters.
   * \return              The cost associated with setting the parameters being optimised to the denoted values.
   */
  float compute_cost(const std::vector<size_t>& valueIndices) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
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
};

}

#endif
