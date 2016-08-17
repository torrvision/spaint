/**
 * evaluation: ParameterSetGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_EVALUATION_PARAMETERSETGENERATOR
#define H_EVALUATION_PARAMETERSETGENERATOR

#include <string>
#include <vector>

#include <boost/spirit/home/support/detail/hold_any.hpp>

#include "../core/ParamSetUtil.h"

namespace evaluation {

/**
 * \brief An instance of a class deriving from this one can be used to generate sets of parameters.
 */
class ParameterSetGenerator
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** A list of the possible values for each parameter (e.g. [("A", [1,2]), ("B", [3,4])]). */
  std::vector<std::pair<std::string,std::vector<boost::spirit::hold_any> > > m_paramValues;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the parameter set generator.
   */
  virtual ~ParameterSetGenerator();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates all possible parameter sets using the Cartesian product approach.
   *
   * \return  The parameter sets.
   */
  virtual std::vector<ParamSet> generate_param_sets() const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds a parameter, together with a list of the values it may take.
   *
   * \param param   The parameter name.
   * \param values  The values the parameter may take.
   * \return        The generator itself (so that calls to add_param may be chained).
   */
  ParameterSetGenerator& add_param(const std::string& param, const std::vector<boost::spirit::hold_any>& values);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a string representation of a parameter set.
   *
   * For example, ["A" -> "1", "B" -> "Foo"] would become "A-1_B-Foo".
   *
   * \param paramSet  The parameter set.
   * \return          A string representation of the parameter set.
   */
  static std::string param_set_to_string(const ParamSet& paramSet);
};

}

#endif
