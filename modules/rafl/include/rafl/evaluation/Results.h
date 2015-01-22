/**
 * rafl: Reuslts.h
 */

#ifndef H_RAFL_RESULTS
#define H_RAFL_RESULTS

#include <utility>
#include <fstream>

#include <boost/spirit/home/support/detail/hold_any.hpp>

#include "ParameterSetProductGenerator.h"
#include "QuantitativePerformance.h"

namespace rafl {

class Results
{
  //#################### PUBLIC TYPEDEFS #################### 
public:
  typedef std::map<std::string,boost::spirit::hold_any> ParamSet;

  //#################### PRIVATE VARIABLES #################### 
private:
  std::vector<std::pair<ParamSet,QuantitativePerformance> > m_results;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs an empty Result.
   */
  Results();

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief Adds a pair of parameters and quantitative performances to the results.
   */
  void push_back(const ParamSet& paramSet, const QuantitativePerformance& QP);

  /**
   * \brief Prints tab delineated results to a stream.
   *
   * \param out     The stream.
   */
  void print_tab_delimited(std::ostream& out) const;
};

}

#endif
