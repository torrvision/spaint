/**
 * evaluation: PerformanceResultUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_PERFORMANCERESULTUTIL
#define H_EVALUATION_PERFORMANCERESULTUTIL

#include <map>

#include "../core/PerformanceResult.h"

namespace evaluation {

/**
 * \brief This struct contains utility functions to make it easier to manipulate performance results.
 */
struct PerformanceResultUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS #################### 

  /**
   * \brief Averages a list of performance results.
   *
   * \param results   The list of performance results.
   * \return          The average of the performance results.
   */
  static PerformanceResult average_results(const std::vector<PerformanceResult>& results);
};

}

#endif
