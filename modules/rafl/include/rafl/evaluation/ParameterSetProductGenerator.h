/**
 * rafl: ParameterSetProductGenerator.h
 */

#ifndef H_RAFL_PARAMETERSETPRODUCTGENERATOR
#define H_RAFL_PARAMETERSETPRODUCTGENERATOR

#include <map>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/spirit/home/support/detail/hold_any.hpp>

namespace rafl {

class ParameterSetProductGenerator
{
  //#################### PUBLIC TYPEDEFS ####################
public:
  typedef std::map<std::string,boost::spirit::hold_any> ParamSet;

  //#################### PRIVATE VARIABLES ####################
private:
std::vector<std::pair<std::string,std::vector<boost::spirit::hold_any> > > m_paramOptions;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  ParameterSetProductGenerator& add_param(const std::string& param, const std::vector<boost::spirit::hold_any>& options);

  std::vector<std::map<std::string,boost::spirit::hold_any> > generate_maps() const;

  static std::string to_string(const ParamSet& params);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  static std::vector<ParamSet> generate_maps_for_param(const std::string& param, const std::vector<boost::spirit::hold_any>& options);

  std::vector<ParamSet> generate_maps_for_params(size_t from) const;
};

}

#endif

