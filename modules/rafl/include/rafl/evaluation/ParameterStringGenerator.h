#ifndef H_RAFL_PARAMETERSTRINGGENERATOR
#define H_RAFL_PARAMETERSTRINGGENERATOR

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;
using boost::spirit::hold_any;

namespace rafl {

class ParameterStringGenerator
{
  typedef std::map<std::string,hold_any> ParamSet;
private:
std::vector<std::pair<std::string,std::vector<hold_any> > > m_paramOptions;

public:
  ParameterStringGenerator& add_param(const std::string& param, const std::vector<hold_any>& options)
  {
    m_paramOptions.push_back(std::make_pair(param, options));
    return *this;
  }

/*  std::vector<std::string> generate() const
  {
    return generate_strings_for_params(0);
  }*/

  std::vector<std::string> generate_strings() const
  {
    return generate_strings_for_params(0);
  }

  std::vector<std::map<std::string,hold_any> > generate_maps() const
  {
    return generate_maps_for_params(0);
  }

  static std::string to_string(const ParamSet& params)
  {
    std::string paramString;
    size_t mapSize = params.size();
    size_t sizeCount = 0;
    for(std::map<std::string,hold_any>::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
    {
      paramString += it->first + "-" + boost::lexical_cast<std::string>(it->second);
      if(++sizeCount != mapSize) paramString += "_";
    }

    return paramString;
  }

private:
  static std::vector<std::string> generate_strings_for_param(const std::string& param, const std::vector<hold_any>& options)
  {
    std::vector<std::string> result;
    for(std::vector<hold_any>::const_iterator it = options.begin(), iend = options.end(); it != iend; ++it)
    {
      result.push_back(param + " " + boost::lexical_cast<std::string>(*it));
    }
    return result;
  }

  std::vector<std::string> generate_strings_for_params(size_t from) const
  {
    if(from == m_paramOptions.size())
    {
      return list_of<std::string>("");
    }
    else
    {
      std::vector<std::string> lhs = generate_strings_for_param(m_paramOptions[from].first, m_paramOptions[from].second);
      std::vector<std::string> rhs = generate_strings_for_params(from + 1);
      std::vector<std::string> result;
      for(size_t i = 0, isize = lhs.size(); i < isize; ++i)
      {
        for(size_t j = 0, jsize = rhs.size(); j < jsize; ++j)
        {
          result.push_back(lhs[i] + " " + rhs[j]);
        }
      }
      return result;
    }
  }


  static std::vector<ParamSet> generate_maps_for_param(const std::string& param, const std::vector<hold_any>& options)
  {
    std::vector<ParamSet> result;
    for(std::vector<hold_any>::const_iterator it = options.begin(), iend = options.end(); it != iend; ++it)
    {
      result.push_back(map_list_of(param, *it));
    }
    return result;
  }

  std::vector<ParamSet> generate_maps_for_params(size_t from) const
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

};

}

#endif

