/**
 * evaluation: ParamSetUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_PARAMSETUTIL
#define H_EVALUATION_PARAMSETUTIL

#include <iosfwd>
#include <map>
#include <string>

#include <boost/assign/list_of.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/export.hpp>

namespace evaluation {

typedef std::map<std::string,std::string> ParamSet;

/**
 * \brief This struct provides utility functions for manipulating parameter sets.
 */
struct ParamSetUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

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
