/**
 * evaluation: PerformanceTable.h
 */

#ifndef H_EVALUATION_PERFORMANCETABLE
#define H_EVALUATION_PERFORMANCETABLE

#include <map>
#include <ostream>
#include <utility>
#include <vector>

#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>

#include "PerformanceMeasure.h"

namespace evaluation {

/**
 * \brief An instance of this class represents a table of performance measures that indicate how well an algorithm performed for various different sets of parameters.
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

  /** The results of running the algorithm with various sets of parameters. */
  std::vector<std::pair<ParamSet,std::map<std::string,PerformanceMeasure> > > m_results;

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
   * \brief Records the performance of the algorithm when run with the specified set of parameters.
   *
   * \param params    The parameters that were used when running the algorithm.
   * \param measures  The performance measures for the algorithm when run with that set of parameters.
   */
  void record_performance(const ParamSet& params, const std::map<std::string,PerformanceMeasure>& measures);
};

}

#endif
