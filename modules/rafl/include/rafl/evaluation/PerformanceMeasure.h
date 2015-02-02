/**
 * rafl: PerformanceMeasure.h
 */

#ifndef H_RAFL_PERFORMANCEMEASURE
#define H_RAFL_PERFORMANCEMEASURE

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace rafl {

/**
 * \brief An instance of this class represents a measure quantifying the performance of an algorithm.
  */
class PerformanceMeasure
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The mean of the measure. */
  float m_mean;

  /** The name of the measure. */
  std::string m_name;

  /** The number of samples used to generate the measure. */
  size_t m_sampleCount;

  /** The standard deviation of the measure. */
  float m_stdDev;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a performance measure from a single sample.
   *
   * \param name  The name of the measure.
   * \param value The performance measured on one sample.
   */
  PerformanceMeasure(const std::string& name, float value)
  : m_mean(value), m_name(name), m_sampleCount(1), m_stdDev(0.0f)
  {}

private:
  /**
   * \brief Constructs an arbitrary performance measure.
   *
   * \param name        The name of the measure.
   * \param sampleCount The number of samples used to calculate the measure.
   * \param mean        The mean of the measure.
   * \param stdDev      The standard deviation of the measure.
   */
  PerformanceMeasure(const std::string& name, size_t sampleCount, float mean, float stdDev)
  : m_mean(mean), m_name(name), m_sampleCount(sampleCount), m_stdDev(stdDev)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the mean of the measure.
   *
   * \return  The mean of the measure.
   */
  float get_mean() const
  {
    return m_mean;
  }

  /**
   * \brief Gets the name of the measure.
   *
   * \return  The name of the measure.
   */
  const std::string& get_name() const
  {
    return m_name;
  }

  /**
   * \brief Gets the number of samples used to generate the measure.
   * 
   * \return  The number of samples used to generate the measure.
   */
  size_t get_sample_count() const
  {
    return m_sampleCount;
  }

  /**
   * \brief Gets the standard deviation of the measure.
   *
   * \return  The standard deviation of the measure.
   */
  float get_std_dev() const
  {
    return m_stdDev;
  }

  /**
   * \brief Gets the variance of the measure.
   *
   * \return  The variance of the measure.
   */
  float get_variance() const
  {
    return m_stdDev * m_stdDev;
  }

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /** 
   * \brief Calculates the average of a set of measures.
   *
   * \param measures  A set of measures (must be non-empty).
   * \return          The average of the measures.
   */
  static PerformanceMeasure average(const std::vector<PerformanceMeasure>& measures)
  {
    // Check that there are measures to average.
    if(measures.empty())
    {
      throw std::runtime_error("Cannot average an empty set of measures");
    }

    // Compute the mean of the combined measure.
    const std::string& measureName = measures[0].get_name();
    float combinedMean = 0.0f;
    size_t combinedSampleCount = 0;
    for(size_t i = 0, size = measures.size(); i < size; ++i)
    {
      if(measures[i].get_name() != measureName)
      {
        throw std::runtime_error("Cannot average a heterogeneous set of measures");
      }

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

    return PerformanceMeasure(measureName, combinedSampleCount, combinedMean, sqrt(combinedVariance));
  }
};

}

#endif
