/**
 * rafl: PerformanceTable.h
 */

#ifndef H_RAFL_PERFORMANCETABLE
#define H_RAFL_PERFORMANCETABLE

#include <map>
#include <ostream>
#include <utility>
#include <vector>

#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>

#include "QuantitativePerformance.h"

namespace rafl {

/**
 * \brief An instance of this class represents a table of performance results that indicate how well an algorithm performed for various different sets of parameters.
 */
class PerformanceTable
{
  //#################### TYPEDEFS ####################
public:
  typedef std::map<std::string,std::string> ParamSet;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The names of the measures (e.g. "Accuracy") that are recorded in the table. */
  std::vector<std::string> m_measureNames;

  /** The quantitative results of the algorithm for the various sets of parameters. */
  std::vector<std::pair<ParamSet,QuantitativePerformance> > m_results;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a performance table.
   *
   * \param measureNames  The names of the measures (e.g. "Accuracy") that are recorded in the table.
   */
  explicit PerformanceTable(const std::vector<std::string>& measureNames);

  /**
   * \brief Constructs a performance table.
   *
   * \param measureNames  The names of the measures (e.g. "Accuracy") that are recorded in the table.
   */
  template <typename T>
  explicit PerformanceTable(const boost::assign_detail::generic_list<T>& measureNames)
  {
    for(typename boost::assign_detail::generic_list<T>::const_iterator it = measureNames.begin(), iend = measureNames.end(); it != iend; ++it)
    {
      m_measureNames.push_back(boost::lexical_cast<std::string>(*it));
    }
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Outputs the table to a stream.
   *
   * \param os        The stream.
   * \param delimiter The delimiter to use to separate the columns of the table.
   */
  void output(std::ostream& os, const std::string& delimiter = "\t") const;

  /**
   * \brief Records the quantitative performance of the algorithm when run with the specified set of parameters.
   *
   * \param params      A set of parameters.
   * \param performance The quantitative performance of the algorithm when run with that set of parameters.
   */
  void record_performance(const ParamSet& params, const QuantitativePerformance& performance);
};

}

#endif
