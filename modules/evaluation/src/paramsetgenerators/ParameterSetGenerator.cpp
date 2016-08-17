/**
 * evaluation: ParameterSetGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "paramsetgenerators/ParameterSetGenerator.h"
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
