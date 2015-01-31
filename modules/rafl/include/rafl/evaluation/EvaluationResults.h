/**
 * rafl: EvaluationResults.h
 */

#ifndef H_RAFL_EVALUATIONRESULTS
#define H_RAFL_EVALUATIONRESULTS

#include <map>
#include <ostream>
#include <utility>
#include <vector>

#include "QuantitativePerformance.h"

namespace rafl {

/**
 * \brief An instance of this class can be used to represent the results of quantitatively evaluating an algorithm.
 *
 * The idea is to compare the performance of the algorithm using various sets of parameters.
 */
class EvaluationResults
{
  //#################### TYPEDEFS ####################
public:
  typedef std::map<std::string,std::string> ParamSet;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The quantitative results of the algorithm for the various sets of parameters. */
  std::vector<std::pair<ParamSet,QuantitativePerformance> > m_results;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Prints the results of evaluating the algorithm to a stream.
   *
   * \param os  The stream.
   */
  void print_tab_delimited(std::ostream& os) const;

  /**
   * \brief Adds a pair of parameters and quantitative performances to the results.
   */
  void push_back(const ParamSet& paramSet, const QuantitativePerformance& QP);
};

}

#endif
