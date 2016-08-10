/**
 * evaluation: PerformanceMeasureUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_EVALUATION_PERFORMANCEMEASUREUTIL
#define H_EVALUATION_PERFORMANCEMEASUREUTIL

#include <map>

#include "../core/PerformanceResult.h"

namespace evaluation {

/**
 * \brief This struct contains utility functions to make it easier to manipulate performance measures and results.
 */
struct PerformanceMeasureUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS #################### 

  /**
   * \brief Calculates the average of a vector of performance measures.
   *
   * \param measures  The vector of measures to average (must be non-empty).
   * \return          The average of the measures.
   */
  static PerformanceMeasure average_measures(const std::vector<PerformanceMeasure>& measures);

  /**
   * \brief Calculates the average of a vector of performance results.
   * 
   * If the performance results contain different performance measures, the average result
   * will contain the averages of all measures present in any input result. For example:
   *
   * Input: [[A->1, B->2]; [A->2, B->1]; [C->3]]
   * Groups: [[A->1, A->2]; [B->2, B->1]; [C->3]]
   * Output: [A->1.5, B->1.5, C->3]
   *
   * \param results   The vector of performance results to average (must be non-empty).
   * \return          The average of the results.
   */
  static PerformanceResult average_results(const std::vector<PerformanceResult>& results);
};

}

#endif
