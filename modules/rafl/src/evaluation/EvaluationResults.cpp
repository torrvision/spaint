/**
 * rafl: EvaluationResults.cpp
 */

#include "evaluation/EvaluationResults.h"

namespace rafl {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void EvaluationResults::print_tab_delimited(std::ostream& os) const
{
  //Get the first set of results, and print the names of the parameters in the set.
  ParamSet firstSet = m_results[0].first;
  ParamSet::iterator it, iend;
  for(it = firstSet.begin(), iend = firstSet.end(); it != iend; ++it)
  {
    os << it->first << "\t";
  }

  //Print the performance measure header.
  m_results[0].second.print_accuracy_header(os);
  os << "\n";

  //Print the results to a stream.
  for(size_t i = 0, iend = m_results.size(); i < iend; ++i)
  {
    ParamSet set = m_results[i].first;
    ParamSet::iterator it, itend;
    for(it = set.begin(), itend = set.end(); it != itend; ++it)
    {
      os << it->second << "\t";
    }
    m_results[i].second.print_accuracy_values(os);
    os << "\n";
  }
}

void EvaluationResults::push_back(const ParamSet& paramSet, const QuantitativePerformance& QP)
{
  m_results.push_back(std::make_pair(paramSet, QP));
}

}
