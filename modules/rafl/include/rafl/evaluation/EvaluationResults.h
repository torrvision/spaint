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
   * \brief Outputs a table showing the results of evaluating the algorithm to a stream.
   *
   * \param os        The stream.
   * \param delimiter The delimiter used to separate the columns of the table.
   */
  void output(std::ostream& os, const std::string& delimiter = "\t") const;

  /**
   * \brief Records the quantitative performance of the algorithm when run with the specified set of parameters.
   *
   * \param params      The set of parameters.
   * \param performance The quantitative performance of the algorithm when run with that set of parameters.
   */
  void record_performance(const ParamSet& params, const QuantitativePerformance& performance);
};

}

#endif
