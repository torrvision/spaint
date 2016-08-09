/**
 * evaluation: PerformanceMeasureUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_PERFORMANCEMEASUREUTIL
#define H_EVALUATION_PERFORMANCEMEASUREUTIL

#include <map>

#include "../core/PerformanceResult.h"

namespace evaluation {

/**
 * \brief This struct contains utility functions to make it easier to manipulate performance results.
 */
struct PerformanceMeasureUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS #################### 

  /**
   * \brief Averages a set of performance results.
   *
   * \param results   The list of performance results.
   * \return          The average of the performance results.
   */
  static PerformanceResult average_results(const std::vector<PerformanceResult>& results);
};

}

#endif
