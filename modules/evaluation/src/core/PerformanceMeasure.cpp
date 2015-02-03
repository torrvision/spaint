/**
 * evaluation: PerformanceMeasure.cpp
 */

#include <ostream>

#include "core/PerformanceMeasure.h"

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

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

PerformanceMeasure PerformanceMeasure::average(const std::vector<PerformanceMeasure>& measures)
{
  // Check that there are measures to average.
  if(measures.empty())
  {
    throw std::runtime_error("Cannot average an empty set of measures");
  }

  // Compute the mean of the combined measure.
  float combinedMean = 0.0f;
  size_t combinedSampleCount = 0;
  for(size_t i = 0, size = measures.size(); i < size; ++i)
  {
    size_t sampleCount = measures[i].get_sample_count();
    combinedMean += sampleCount * measures[i].get_mean();
    combinedSampleCount += sampleCount;
  }
  combinedMean /= combinedSampleCount;

  // Compute the variance of the combined measure.
  float combinedVariance = 0.0f;
  for(size_t i = 0, size = measures.size(); i < size; ++i)
  {
    float delta = measures[i].get_mean() - combinedMean; 
    combinedVariance += measures[i].get_sample_count() * (measures[i].get_variance() + delta * delta);
  }
  combinedVariance /= combinedSampleCount;

  return PerformanceMeasure(combinedSampleCount, combinedMean, sqrt(combinedVariance));
}

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const PerformanceMeasure& rhs)
{
  os << rhs.get_mean() << " +/- " << rhs.get_std_dev() << " (" << rhs.get_sample_count() << " samples)";
  return os;
}

}

