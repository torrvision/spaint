/**
 * evaluation: PerformanceMeasure.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_PERFORMANCEMEASURE
#define H_EVALUATION_PERFORMANCEMEASURE

#include <cstddef>
#include <iosfwd>
#include <vector>

namespace evaluation {

/**
 * \brief An instance of this class represents a measure quantifying the performance of an algorithm.
 */
class PerformanceMeasure
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The mean of the measure. */
  float m_mean;

  /** The number of samples used to generate the measure. */
  size_t m_sampleCount;

  /** The standard deviation of the measure. */
  float m_stdDev;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a performance measure from a single sample.
   *
   * \param value The performance as measured on a single sample.
   */
  PerformanceMeasure(float value);

  /**
   * \brief Constructs an arbitrary performance measure.
   *
   * \param sampleCount The number of samples used to calculate the measure.
   * \param mean        The mean of the measure.
   * \param stdDev      The standard deviation of the measure.
   */
  PerformanceMeasure(size_t sampleCount, float mean, float stdDev);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the mean of the measure.
   *
   * \return  The mean of the measure.
   */
  float get_mean() const;

  /**
   * \brief Gets the number of samples used to generate the measure.
   * 
   * \return  The number of samples used to generate the measure.
   */
  size_t get_sample_count() const;

  /**
   * \brief Gets the standard deviation of the measure.
   *
   * \return  The standard deviation of the measure.
   */
  float get_std_dev() const;

  /**
   * \brief Gets the variance of the measure.
   *
   * \return  The variance of the measure.
   */
  float get_variance() const;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /** 
   * \brief Calculates the average of a set of measures.
   *
   * \param measures  A set of measures (must be non-empty).
   * \return          The average of the measures.
   */
  static PerformanceMeasure average(const std::vector<PerformanceMeasure>& measures);
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a performance measure to a stream.
 *
 * \param os  The stream.
 * \param rhs The performance measure.
 * \return    The stream.
 */
std::ostream& operator<<(std::ostream& os, const PerformanceMeasure& rhs);

}

#endif
