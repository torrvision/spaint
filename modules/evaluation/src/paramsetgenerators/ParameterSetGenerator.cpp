/**
 * evaluation: ParameterSetGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "paramsetgenerators/ParameterSetGenerator.h"

#include <boost/lexical_cast.hpp>
using boost::spirit::hold_any;

namespace evaluation {

//#################### DESTRUCTOR ####################

ParameterSetGenerator::~ParameterSetGenerator() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ParameterSetGenerator& ParameterSetGenerator::add_param(const std::string& param, const std::vector<hold_any>& values)
{
  m_paramValues.push_back(std::make_pair(param, values));
  return *this;
}

void ParameterSetGenerator::initialise()
{
  // No-op by default
}

std::string ParameterSetGenerator::param_values_to_string() const
{
  std::string result;
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

    result += paramString + '\n';
  }
  return result;
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::string ParameterSetGenerator::param_set_to_string(const ParamSet& params)
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

}
