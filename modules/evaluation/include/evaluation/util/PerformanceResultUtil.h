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
   * Note: If the performance results are non-homogeneous,
   *       it will return the union of the performance measures:
   *
   * Example: Input: { [A->1, B->2]; [A->2, B->1]; [C->3] }
   *
   *         Groups: { [A->1, A->2]; [B->2, B->1]; [C->3] }
   *
   *         Output: [A->1.5, B->1.5, C->3].
   *
   * \param results   The list of performance results.
   * \return          The average of the performance results.
   */
  static PerformanceResult average_results(const std::vector<PerformanceResult>& results);
};

}

#endif
