/**
 * rafl: QuantitativePerformance.h
 */

#ifndef H_RAFL_QUANTITATIVEPERFORMANCE
#define H_RAFL_QUANTITATIVEPERFORMANCE

namespace rafl {

class QuantitativePerformance
{
private:
  std::pair<float,float> m_accuracy;
  size_t m_samples;

public:
  explicit QuantitativePerformance(float accuracy)
  : m_accuracy(std::make_pair(accuracy, 0.0f)), m_samples(1)
  {
  }
  explicit QuantitativePerformance(const std::vector<QuantitativePerformance> QP)
  : m_samples(QP.size())
  {
    //return std::accumulate(QP.begin(), QP.end(), 0.0)/QP.size();
    float sumMean = 0;
    for(size_t i = 0; i < m_samples; ++i)
    {
      sumMean += QP[i].mean_accuracy();
    }
    float mean = sumMean/m_samples;
    m_accuracy.first = mean;

    float sumVariance = 0;
    for(size_t i = 0; i < m_samples; ++i)
    {
      sumVariance += pow(mean - QP[i].mean_accuracy(), 2);
    }
    m_accuracy.second = sqrt(sumVariance/m_samples);
  }

  float mean_accuracy() const
  {
    return m_accuracy.first;
  }

  std::ostream& print_accuracy_header(std::ostream& out) const
  {
    out << "Acc" << "\t" << "Std";
    return out;
  }

  std::ostream& print_accuracy_values(std::ostream& out) const
  {
    out << m_accuracy.first << "\t" << m_accuracy.second;
    return out;
  }

  friend std::ostream& operator<<(std::ostream& out, const QuantitativePerformance& qp)
  {
    out << "accuracy: " << qp.m_accuracy.first << " +/- " << qp.m_accuracy.second << ", samples: " << qp.m_samples << "\n";
    return out;
  }
};

}

#endif

