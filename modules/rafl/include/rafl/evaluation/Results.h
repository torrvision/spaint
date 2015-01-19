/**
 * rafl: Reuslts.h
 */

#ifndef H_RAFL_RESULTS
#define H_RAFL_RESULTS

#include <utility>
#include <fstream>
#include <boost/spirit/home/support/detail/hold_any.hpp>

#include "ParameterStringGenerator.h"
#include "QuantitativePerformance.h"

namespace rafl {
class Results
{
  typedef std::map<std::string,boost::spirit::hold_any> ParamSet;

  private:
    std::vector<std::pair<ParamSet,QuantitativePerformance> > m_results;
    //std::map<ParamSet,QuantitativePerformance> m_results;

  public:
    Results(){}

  public:
    void push_back(const ParamSet& paramSet, const QuantitativePerformance& QP)
    {
      m_results.push_back(std::make_pair(paramSet, QP));
    }

    void print_tab_delimited(const std::string& path)
    {
      std::ofstream cout(path);
      if(!cout)
      {
        cout << "Warning could not open file for writing..\n";
      }
      else{
        std::cout << "Writing files to: " << path << "\n";

        ParamSet firstSet = m_results[0].first;
        ParamSet::iterator it, iend;
        for(it = firstSet.begin(), iend = firstSet.end(); it != iend; ++it)
        {
          cout << it->first << "\t";
        }
        m_results[0].second.print_accuracy_header(cout);
        cout << "\n";

        for(size_t i = 0, iend = m_results.size(); i < iend; ++i)
        {

          ParamSet set = m_results[i].first;
          ParamSet::iterator it, itend;
          for(it = set.begin(), itend = set.end(); it != itend; ++it)
          {
            cout << it->second << "\t";
          }
          m_results[i].second.print_accuracy_values(cout);
          //cout << m_results[i].second;

          cout << "\n";
        }
      }
    }

};
}

#endif
