/**
 * evaluation: PerformanceResult.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_EVALUATION_PERFORMANCERESULT
#define H_EVALUATION_PERFORMANCERESULT

#include "PerformanceMeasure.h"

#include <map>

namespace evaluation {

typedef std::map<std::string,PerformanceMeasure> PerformanceResult;

}

#endif
