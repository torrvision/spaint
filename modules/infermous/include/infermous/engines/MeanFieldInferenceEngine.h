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

  /** TODO */
  std::vector<Eigen::Vector2i> m_neighbourOffsets;

  /** TODO */
  PotentialsGrid_Ptr m_newMarginals;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a mean-field inference engine.
   *
   * \param crf               The CRF on which the mean-field inference engine works.
   * \param neighbourOffsets  TODO
   */
  MeanFieldInferenceEngine(const CRF2D_Ptr& crf, const std::vector<Eigen::Vector2i>& neighbourOffsets)
  : m_crf(crf), m_neighbourOffsets(neighbourOffsets), m_newMarginals(new PotentialsGrid(crf->get_height(), crf->get_width()))
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
   */
  void update_crf()
  {
    for(int y = 0, height = m_crf->get_height(); y < height; ++y)
    {
      for(int x = 0, width = m_crf->get_width(); x < width; ++x)
      {
        update_pixel(Eigen::Vector2i(x, y));
      }
    }

    // Replace the marginals with the new marginals and update the time step of the CRF.
    m_crf->swap_marginals(m_newMarginals);
    m_crf->increment_time_step();
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  float compute_M_i_L(const Eigen::Vector2i& i, const Label& L, float phi_i_L) const
  {
    float result = phi_i_L;

    for(std::vector<Eigen::Vector2i>::const_iterator nt = m_neighbourOffsets.begin(), nend = m_neighbourOffsets.end(); nt != nend; ++nt)
    {
      Eigen::Vector2i j = i + *nt;
      if(!m_crf->within_bounds(j)) continue;
      const std::map<Label,float>& Q_j = m_crf->get_marginals_at(j);
      for(typename std::map<Label,float>::const_iterator kt = Q_j.begin(), kend = Q_j.end(); kt != kend; ++kt)
      {
        const Label& LDash = kt->first;
        float Q_j_LDash = kt->second;
        float phi_i_j_L_LDash = m_crf->get_pairwise_potential_calculator()->calculate_potential(L, LDash);
        result += Q_j_LDash * phi_i_j_L_LDash;
      }
    }

    return result;
  }

  /**
   * \brief Updates the specified pixel in the CRF.
   *
   * \param i The location of the pixel to update.
   */
  void update_pixel(const Eigen::Vector2i& i)
  {
    // Get the unary potentials for the pixel.
    const std::map<Label,float>& phi_i = m_crf->get_unaries_at(i);

    // TODO: Add a comment.
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

    // TODO: Add a comment.
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
