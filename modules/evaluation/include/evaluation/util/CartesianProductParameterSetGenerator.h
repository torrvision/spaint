/**
 * evaluation: CartesianProductParameterSetGenerator.h
 */

#ifndef H_EVALUATION_CARTESIANPRODUCTPARAMETERSETGENERATOR
#define H_EVALUATION_CARTESIANPRODUCTPARAMETERSETGENERATOR

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <boost/spirit/home/support/detail/hold_any.hpp>

namespace evaluation {

/**
 * \brief An instance of this class can be used to generate sets of parameters using a Cartesian product approach.
 *
 * Example:
 *
 * A in {1,2}, B in {3,4}, C in {5,6,7} -> (A,B,C) in {(1,3,5), (1,3,6), (1,3,7), (1,4,5), ..., (2,4,7)}
 */
class CartesianProductParameterSetGenerator
{
  //#################### PUBLIC TYPEDEFS ####################
public:
  typedef std::map<std::string,std::string> ParamSet;

  //#################### PRIVATE VARIABLES ####################
private:
  std::vector<std::pair<std::string,std::vector<boost::spirit::hold_any> > > m_paramOptions;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  CartesianProductParameterSetGenerator& add_param(const std::string& param, const std::vector<boost::spirit::hold_any>& options);

  std::vector<ParamSet> generate_maps() const;

  static std::string to_string(const ParamSet& params);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  static std::vector<ParamSet> generate_maps_for_param(const std::string& param, const std::vector<boost::spirit::hold_any>& options);

  std::vector<ParamSet> generate_maps_for_params(size_t from) const;
};

}

#endif
