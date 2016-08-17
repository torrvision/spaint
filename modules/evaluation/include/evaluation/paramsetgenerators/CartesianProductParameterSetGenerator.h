/**
 * evaluation: CartesianProductParameterSetGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_CARTESIANPRODUCTPARAMETERSETGENERATOR
#define H_EVALUATION_CARTESIANPRODUCTPARAMETERSETGENERATOR

#include "ParameterSetGenerator.h"

namespace evaluation {

/**
 * \brief An instance of this class can be used to generate sets of parameters using a Cartesian product approach.
 *
 * Example:
 *
 * A in {1,2}, B in {3,4}, C in {5,6,7} -> (A,B,C) in {(1,3,5), (1,3,6), (1,3,7), (1,4,5), ..., (2,4,7)}
 */
class CartesianProductParameterSetGenerator : public ParameterSetGenerator
{
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual std::vector<ParamSet> generate_param_sets() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Generates the partial parameter sets for an individual parameter.
   *
   * For example, A in {1,2} would become [["A" -> "1"], ["A" -> "2"]].
   *
   * \param param   The parameter name.
   * \param values  The values the parameter may take.
   * \return        The partial parameter sets for the specified parameter.
   */
  static std::vector<ParamSet> generate_partial_param_sets_for_param(const std::string& param, const std::vector<boost::spirit::hold_any>& values);

  /**
   * \brief Generates the partial parameter sets for the parameters whose index is >= from.
   *
   * \param from  The lower bound of the range of parameters for which to build partial parameter sets.
   * \return      The partial parameter sets for the parameters whose index is >= from.
   */
  std::vector<ParamSet> generate_partial_param_sets_for_params(size_t from) const;
};

}

#endif
