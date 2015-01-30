/**
 * rafl: ParameterSetProductGenerator.cpp
 */

#include "evaluation/ParameterSetProductGenerator.h"

#include <boost/assign.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;
using boost::spirit::hold_any;

namespace rafl {

//#################### PUBLIC MEMBER FUNCTIONS ####################
ParameterSetProductGenerator& ParameterSetProductGenerator::add_param(const std::string& param, const std::vector<boost::spirit::hold_any>& options)
{
  m_paramOptions.push_back(std::make_pair(param, options));
  return *this;
}

std::vector<ParameterSetProductGenerator::ParamSet> ParameterSetProductGenerator::generate_maps() const
{
  return generate_maps_for_params(0);
}

std::string ParameterSetProductGenerator::to_string(const ParamSet& params)
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
std::vector<ParameterSetProductGenerator::ParamSet> ParameterSetProductGenerator::generate_maps_for_param(const std::string& param, const std::vector<hold_any>& options)
{
  std::vector<ParamSet> result;
  for(std::vector<hold_any>::const_iterator it = options.begin(), iend = options.end(); it != iend; ++it)
  {
    result.push_back(map_list_of(param, boost::lexical_cast<std::string>(*it)));
  }
  return result;
}

std::vector<ParameterSetProductGenerator::ParamSet> ParameterSetProductGenerator::generate_maps_for_params(size_t from) const
{
  if(from == m_paramOptions.size())
  {
    return list_of(ParamSet());
  }
  else
  {
    std::vector<ParamSet> lhs = generate_maps_for_param(m_paramOptions[from].first, m_paramOptions[from].second);
    std::vector<ParamSet> rhs = generate_maps_for_params(from + 1);
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

} //end namespace rafl

