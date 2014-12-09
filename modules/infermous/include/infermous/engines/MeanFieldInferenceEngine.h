/**
 * infermous: MeanFieldInferenceEngine.h
 */

#ifndef H_INFERMOUS_MEANFIELDINFERENCEENGINE
#define H_INFERMOUS_MEANFIELDINFERENCEENGINE

#include <vector>

#include "../base/CRF2D.h"

namespace infermous {

/**
 * \brief An instance of an instantiation of this class template can be used to run mean-field inference on a 2D CRF.
 */
template <typename Label>
class MeanFieldInferenceEngine
{
  //#################### TYPEDEFS ####################
public:
  typedef CRF2D_Ptr<Label> CRF2D_Ptr;
  typedef CRF2D_CPtr<Label> CRF2D_CPtr;
  typedef PotentialsGrid<Label> PotentialsGrid;
  typedef PotentialsGrid_Ptr<Label> PotentialsGrid_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The CRF on which the mean-field inference engine works. */
  CRF2D_Ptr m_crf;

  /** A list of offsets used to specify the neighbours of each pixel. */
  std::vector<Eigen::Vector2i> m_neighbourOffsets;

  /** A grid of updated marginal potentials that will be swapped with the grid in the CRF at the end of each time step. */
  PotentialsGrid_Ptr m_newMarginals;

  /** The pairwise potential calculator. */
  PairwisePotentialCalculator_CPtr<Label> m_pairwisePotentialCalculator;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a mean-field inference engine.
   *
   * \param crf               The CRF on which the mean-field inference engine works.
   * \param neighbourOffsets  A list of offsets used to specify the neighbours of each pixel.
   */
  MeanFieldInferenceEngine(const CRF2D_Ptr& crf, const std::vector<Eigen::Vector2i>& neighbourOffsets)
  : m_crf(crf),
    m_neighbourOffsets(neighbourOffsets),
    m_newMarginals(new PotentialsGrid(crf->get_height(), crf->get_width())),
    m_pairwisePotentialCalculator(crf->get_pairwise_potential_calculator())
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the CRF on which the mean-field inference engine works.
   *
   * \return  The CRF on which the mean-field inference engine works.
   */
  CRF2D_CPtr get_crf() const
  {
    return m_crf;
  }

  /**
   * \brief Updates the CRF on which the mean-field inference engine works.
   *
   * \param iterations  The number of update iterations to run.
   */
  void update_crf(size_t iterations)
  {
    for(size_t i = 0; i < iterations; ++i)
    {
      for(int y = 0, height = m_crf->get_height(); y < height; ++y)
      {
        for(int x = 0, width = m_crf->get_width(); x < width; ++x)
        {
          compute_updated_pixel(Eigen::Vector2i(x, y));
        }
      }

      // Swap the new marginals into the CRF and update its time step.
      m_crf->swap_marginals(m_newMarginals);
      m_crf->increment_time_step();
    }
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes the value of M_i(L), for pixel i and label L.
   *
   * See p.6 of the original SemanticPaint paper for details - essentially, this is the negative log of the unnormalised new potential for the pixel.
   *
   * \param i       The location of the pixel.
   * \param L       The label.
   * \param phi_i_L The unary potential, phi_i(L), for the specified pixel and label (we pass this in to avoid having to recalculate it).
   * \return        The value of M_i(L) for the specified pixel and label.
   */
  float compute_M_i_L(const Eigen::Vector2i& i, const Label& L, float phi_i_L) const
  {
    float result = phi_i_L;

    // Note that we have reordered the sums presented in the paper so as to iterate over the neighbours first.
    // This makes no difference to the result, but allows us to avoid lookups of Q_j^{t-1}(L').
    for(std::vector<Eigen::Vector2i>::const_iterator nt = m_neighbourOffsets.begin(), nend = m_neighbourOffsets.end(); nt != nend; ++nt)
    {
      // Calculate the location of the potential neighbour and check whether or not it is within the CRF. If not, skip it.
      Eigen::Vector2i j = i + *nt;
      if(!m_crf->within_bounds(j)) continue;

      // Calculate \sum_{L'} (Q_j^{t-1}(L') * phi_ij(L,L')).
      const std::map<Label,float>& Q_j = m_crf->get_marginals_at(j);
      for(typename std::map<Label,float>::const_iterator kt = Q_j.begin(), kend = Q_j.end(); kt != kend; ++kt)
      {
        const Label& LDash = kt->first;
        float Q_j_LDash = kt->second;
        float phi_i_j_L_LDash = m_pairwisePotentialCalculator->calculate_potential(L, LDash);
        result += Q_j_LDash * phi_i_j_L_LDash;
      }
    }

    return result;
  }

  /**
   * \brief Computes the updated version of the specified pixel in the CRF.
   *
   * \param i The location of the pixel whose updated version we want to compute.
   */
  void compute_updated_pixel(const Eigen::Vector2i& i)
  {
    // Get the unary potentials for the pixel.
    const std::map<Label,float>& phi_i = m_crf->get_unaries_at(i);

    // Calculate the unnormalised new potentials for the pixel, together with the normalisation constant.
    // (In other words, compute e^-M_i(L) for every L, and Z_i, as per the original SemanticPaint paper.)
    std::map<Label,float> oneOverE_M_i;
    float Z_i = 0.0f;
    for(typename std::map<Label,float>::const_iterator kt = phi_i.begin(), kend = phi_i.end(); kt != kend; ++kt)
    {
      const Label& L = kt->first;
      float phi_i_L = kt->second;
      float oneOverE_M_i_L = expf(-compute_M_i_L(i, L, phi_i_L));
      oneOverE_M_i[L] = oneOverE_M_i_L;
      Z_i += oneOverE_M_i_L;
    }

    // Calculate the normalised new potentials for the pixel by dividing through by the normalisation constant.
    // (In other words, compute Q_i^t(L) = 1/Z_i * e^-M_i(L), as per the paper.)
    float oneOverZ_i = 1.0f / Z_i;
    std::map<Label,float>& Q_i = (*m_newMarginals)(i.y(), i.x());
    for(typename std::map<Label,float>::const_iterator kt = oneOverE_M_i.begin(), kend = oneOverE_M_i.end(); kt != kend; ++kt)
    {
      const Label& L = kt->first;
      float oneOverE_M_i_L = kt->second;
      Q_i[L] = oneOverZ_i * oneOverE_M_i_L;
    }
  }
};

}

#endif
