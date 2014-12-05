/**
 * infermous: MeanFieldInferenceEngine.h
 */

#ifndef H_INFERMOUS_MEANFIELDINFERENCEENGINE
#define H_INFERMOUS_MEANFIELDINFERENCEENGINE

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
  typedef boost::shared_ptr<CRF2D<Label> > CRF2D_Ptr;
  typedef typename CRF2D<Label>::ProbabilitiesGrid ProbabilitiesGrid;
  typedef boost::shared_ptr<ProbabilitiesGrid> ProbabilitiesGrid_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The CRF on which the mean-field inference engine works. */
  CRF2D_Ptr m_crf;

  /** TODO */
  std::vector<Eigen::Vector2i> m_neighbourOffsets;

  /** TODO */
  ProbabilitiesGrid_Ptr m_newMarginals;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a mean-field inference engine.
   *
   * \param crf             The CRF on which the mean-field inference engine works.
   * \param neighbourRadius TODO
   */
  MeanFieldInferenceEngine(const CRF2D_Ptr& crf, int neighbourRadius)
  : m_crf(crf), m_newMarginals(new ProbabilitiesGrid(crf->get_height(), crf->get_width()))
  {
    // TODO: Write a comment.
    float neighbourRadiusSquared = static_cast<float>(neighbourRadius * neighbourRadius);
    for(int y = -neighbourRadius; y <= neighbourRadius; ++y)
    {
      for(int x = -neighbourRadius; x <= neighbourRadius; ++x)
      {
        if(x == 0 && y == 0) continue;

        float distanceSquared = static_cast<float>(x*x + y*y);
        if(distanceSquared <= neighbourRadiusSquared)
        {
          m_neighbourOffsets.push_back(Eigen::Vector2i(x,y));
        }
      }
    }
  }

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
    for(int row = 0, height = m_crf->get_height(); row < height; ++row)
    {
      for(int col = 0, width = m_crf->get_width(); col < width; ++col)
      {
        update_pixel(row, col);
      }
    }

    // Replace the marginals with the new marginals and update the time step of the CRF.
    m_crf->set_marginals(m_newMarginals);
    m_crf->increment_time_step();
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  float compute_M_i_L(int row, int col, const Label& L, float phi_i_L) const
  {
    float result = phi_i_L;

    const std::map<Label,float>& phi_i = m_crf->get_unaries(row, col);
    for(typename std::map<Label,float>::const_iterator it = phi_i.begin(), iend = phi_i.end(); it != iend; ++it)
    {
      const Label& LDash = it->first;
      for(std::vector<Eigen::Vector2i>::const_iterator jt = m_neighbourOffsets.begin(), jend = m_neighbourOffsets.end(); jt != jend; ++jt)
      {
        int x = col + jt->x(), y = row + jt->y();
        if(!m_crf->within_bounds(row, col)) continue;

        const std::map<Label,float>& Q_j = m_crf->get_marginals(y, x);
        float Q_j_LDash = Q_j.find(LDash)->second;
        float phi_i_j_L_LDash = m_crf->get_pairwise_potential_calculator()->calculate_potential(L, LDash);
        result += Q_j_LDash * phi_i_j_L_LDash;
      }
    }

    return result;
  }

  /**
   * \brief Updates the specified pixel in the CRF.
   *
   * \param row The row of the pixel to update.
   * \param col The column of the pixel to update.
   */
  void update_pixel(int row, int col)
  {
    // Get the unary potentials for the pixel.
    const std::map<Label,float>& phi_i = m_crf->get_unaries(row, col);

    // TODO: Add a comment.
    std::map<Label,float> M_i;
    float Z_i = 0.0f;
    for(typename std::map<Label,float>::const_iterator it = phi_i.begin(), iend = phi_i.end(); it != iend; ++it)
    {
      const Label& L = it->first;
      float phi_i_L = it->second;
      float M_i_L = compute_M_i_L(row, col, L, phi_i_L);
      M_i[L] = M_i_L;
      Z_i += expf(-M_i_L);
    }

    // TODO: Add a comment.
    float oneOverZ_i = 1.0f / Z_i;
    std::map<Label,float>& Q_i = (*m_newMarginals)(row, col);
    for(typename std::map<Label,float>::const_iterator it = M_i.begin(), iend = M_i.end(); it != iend; ++it)
    {
      const Label& L = it->first;
      float M_i_L = it->second;
      Q_i[L] = oneOverZ_i * exp(-M_i_L);
    }
  }
};

}

#endif
