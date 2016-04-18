/**
 * evaluation: PerformanceMeasureUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_PERFORMANCEMEASUREUTIL
#define H_EVALUATION_PERFORMANCEMEASUREUTIL

#include <map>

#include "../core/PerformanceMeasure.h"

namespace evaluation {

/**
 * \brief This struct contains utility functions to make it easier to manipulate performance measures.
 */
struct PerformanceMeasureUtil
{
  //#################### PUBLIC STATIC MEMBER FNCTIONS #################### 

  /**
   * \brief Averages a set of performance measures.
   *
   * \param measures  The list of performance measures.
   * \return          The average of the performance measures.
   */
  static std::map<std::string,PerformanceMeasure> average_results(const std::vector<std::map<std::string,PerformanceMeasure> >& measures);
};

}

#endif
