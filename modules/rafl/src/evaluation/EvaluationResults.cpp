/**
 * rafl: EvaluationResults.cpp
 */

#include "evaluation/EvaluationResults.h"

#include <string>

namespace rafl {

//#################### PUBLIC MEMBER FUNCTIONS ####################

void EvaluationResults::output(std::ostream& os, const std::string& delimiter) const
{
  // Output the column titles (the parameter names and performance measures).
  const ParamSet& firstSet = m_results[0].first;
  for(ParamSet::const_iterator it = firstSet.begin(), iend = firstSet.end(); it != iend; ++it)
  {
    os << it->first << delimiter;
  }
  m_results[0].second.print_accuracy_header(os);
  os << '\n';

  // Output the results of running the algorithm with different sets of parameters (one set per row).
  for(size_t i = 0, size = m_results.size(); i < size; ++i)
  {
    const ParamSet& params = m_results[i].first;
    for(ParamSet::const_iterator jt = params.begin(), jend = params.end(); jt != jend; ++jt)
    {
      os << jt->second << delimiter;
    }
    m_results[i].second.print_accuracy_values(os);
    os << '\n';
  }
}

void EvaluationResults::record_performance(const ParamSet& params, const QuantitativePerformance& performance)
{
  m_results.push_back(std::make_pair(params, performance));
}

}
