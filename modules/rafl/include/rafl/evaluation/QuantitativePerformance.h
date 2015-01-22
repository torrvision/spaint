/**
 * rafl: QuantitativePerformance.h
 */

#ifndef H_RAFL_QUANTITATIVEPERFORMANCE
#define H_RAFL_QUANTITATIVEPERFORMANCE

namespace rafl {

/**
 * \brief An instance of this class represents the quantitative performance obtained by an algorithm.
 */
class QuantitativePerformance
{
  //#################### NESTED TYPES #################### 
private:
  /**
   * \brief An instance of this struct represents a performance measure.
   */
  struct Measure
  {
    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VAIRABLES ~~~~~~~~~~~~~~~~~~~~ 

    /** The mean of the measure. */
    float mean;

    /** The standard deviation of the measure. */
    float stdDev;
  };

  /** The accuracy measure. */
  Measure m_accuracy;

  /** The number of samples used to generate the performance measures. */
  size_t m_samples;

  //#################### CONSTRUCTORS ####################  
public:
  /**
   * \brief Constructs a quantitative performance measure from a single sample.
   *
   * \param accuracy   The accuracy measured from one sample.
   */
  explicit QuantitativePerformance(float accuracy)
  : m_samples(1)
  {
    m_accuracy.mean = accuracy; 
    m_accuracy.stdDev = 0.0f;
  }

  /**
   * \brief Constructs a single quantitative perofrmance measure form a vector of them.
   *
   * \param qpv    A vector of quantitative performance measures. 
   */
  explicit QuantitativePerformance(const std::vector<QuantitativePerformance> qpv)
  : m_samples(qpv.size())
  {
    float sumMean = 0;
    for(size_t i = 0; i < m_samples; ++i)
    {
      sumMean += qpv[i].mean_accuracy();
    }
    m_accuracy.mean = sumMean / m_samples;

    float sumVariance = 0;
    for(size_t i = 0; i < m_samples; ++i)
    {
      sumVariance += pow(m_accuracy.mean - qpv[i].mean_accuracy(), 2);
    }
    m_accuracy.stdDev = sqrt(sumVariance / m_samples);
  }

  /**
   * \brief Getter function for mean accuracy. 
   *
   * \return The mean accuracy measure. 
   */
  float mean_accuracy() const
  {
    return m_accuracy.mean;
  }

  /** 
   * \brief Prints a tab delineated header to a stream.
   *
   * \param out  The stream.
   * \return     The stream.
   */
  std::ostream& print_accuracy_header(std::ostream& out) const
  {
    out << "Acc" << "\t" << "Std";
    return out;
  }

  /**
   * \brief Prints tab delineated accuracy values to a stream.
   *
   * \param out  The stream.
   * \return     The stream.
   */
  std::ostream& print_accuracy_values(std::ostream& out) const
  {
    out << m_accuracy.mean << "\t" << m_accuracy.stdDev;
    return out;
  }

  /**
   * \brief Binds a quantitative performance object to a stream.
   *
   * \param out   The stream.
   * \param qp    The quantitative performance object.
   * \return      The stream.
   */
  friend std::ostream& operator<<(std::ostream& out, const QuantitativePerformance& qp)
  {
    out << "accuracy: " << qp.m_accuracy.mean << " +/- " << qp.m_accuracy.stdDev << ", samples: " << qp.m_samples << "\n";
    return out;
  }
};

} //end namespace rafl

#endif

