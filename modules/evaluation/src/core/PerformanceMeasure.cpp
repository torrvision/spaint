/**
 * evaluation: PerformanceMeasure.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "core/PerformanceMeasure.h"

#include <ostream>

namespace evaluation {

//#################### CONSTRUCTORS ####################

PerformanceMeasure::PerformanceMeasure(float value)
: m_mean(value), m_sampleCount(1), m_stdDev(0.0f)
{}

PerformanceMeasure::PerformanceMeasure(size_t sampleCount, float mean, float stdDev)
: m_mean(mean), m_sampleCount(sampleCount), m_stdDev(stdDev)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

float PerformanceMeasure::get_mean() const
{
  return m_mean;
}

size_t PerformanceMeasure::get_sample_count() const
{
  return m_sampleCount;
}

float PerformanceMeasure::get_std_dev() const
{
  return m_stdDev;
}

float PerformanceMeasure::get_variance() const
{
  return m_stdDev * m_stdDev;
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const PerformanceMeasure& rhs)
{
  os << rhs.get_mean() << " +/- " << rhs.get_std_dev() << " (" << rhs.get_sample_count() << " samples)";
  return os;
}

}
