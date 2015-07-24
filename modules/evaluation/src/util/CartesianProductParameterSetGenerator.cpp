/**
 * evaluation: CartesianProductParameterSetGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "util/CartesianProductParameterSetGenerator.h"

#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;
using boost::spirit::hold_any;

namespace evaluation {

//#################### PUBLIC MEMBER FUNCTIONS ####################

CartesianProductParameterSetGenerator& CartesianProductParameterSetGenerator::add_param(const std::string& param, const std::vector<hold_any>& values)
{
  m_paramValues.push_back(std::make_pair(param, values));
  return *this;
}

std::vector<CartesianProductParameterSetGenerator::ParamSet> CartesianProductParameterSetGenerator::generate_param_sets() const
{
  return generate_partial_param_sets_for_params(0);
}

std::string CartesianProductParameterSetGenerator::param_set_to_string(const ParamSet& params)
{
  std::string paramString;
  size_t mapSize = params.size();
  size_t sizeCount = 0;
  for(std::map<std::string,std::string>::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
  {
    paramString += it->first + "-" + it->second;
    if(++sizeCount != mapSize) paramString += "_";
  }

  return paramString;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

std::vector<CartesianProductParameterSetGenerator::ParamSet>
CartesianProductParameterSetGenerator::generate_partial_param_sets_for_param(const std::string& param, const std::vector<hold_any>& values)
{
  std::vector<ParamSet> result;
  for(std::vector<hold_any>::const_iterator it = values.begin(), iend = values.end(); it != iend; ++it)
  {
    result.push_back(map_list_of(param, boost::lexical_cast<std::string>(*it)));
  }
  return result;
}

std::vector<CartesianProductParameterSetGenerator::ParamSet>
CartesianProductParameterSetGenerator::generate_partial_param_sets_for_params(size_t from) const
{
  if(from == m_paramValues.size())
  {
    return list_of(ParamSet());
  }
  else
  {
    std::vector<ParamSet> lhs = generate_partial_param_sets_for_param(m_paramValues[from].first, m_paramValues[from].second);
    std::vector<ParamSet> rhs = generate_partial_param_sets_for_params(from + 1);
    std::vector<ParamSet> result;
    for(size_t i = 0, isize = lhs.size(); i < isize; ++i)
    {
      for(size_t j = 0, jsize = rhs.size(); j < jsize; ++j)
      {
        ParamSet combinedParameters = lhs[i];
        std::copy(rhs[j].begin(), rhs[j].end(), std::inserter(combinedParameters, combinedParameters.begin()));
        result.push_back(combinedParameters);
      }
    }
    return result;
  }
}

}
