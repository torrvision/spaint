/**
 * infermous: CRF2D.h
 */

#ifndef H_INFERMOUS_CRF2D
#define H_INFERMOUS_CRF2D

#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <tvgutil/LimitedContainer.h>

#define PRT(X) std::cout << "#X: " << X << std::endl;

namespace infermous {

template <typename Label>
class CRF2D
{
  //#################### TYPEDEFS ####################
public:
  typedef Eigen::Matrix<std::map<Label,float>,-1,-1> ProbabilitiesGrid;
  typedef boost::shared_ptr<ProbabilitiesGrid> ProbabilitiesGrid_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  ProbabilitiesGrid_Ptr m_marginals;
  std::vector<Eigen::Vector2i> m_neighbourOffsets;
  ProbabilitiesGrid_Ptr m_newMarginals;
  size_t m_time;
  ProbabilitiesGrid_Ptr m_unaries;
  size_t m_width, m_height;

  //#################### CONSTRUCTORS ####################
public:
  CRF2D(const ProbabilitiesGrid_Ptr& unaries, int neighbourRadius)
  : m_height(unaries->rows()), m_width(unaries->cols()), m_time(0), m_unaries(unaries)
  {
    m_marginals.reset(new ProbabilitiesGrid(*unaries));
    m_newMarginals.reset(new ProbabilitiesGrid(unaries->rows(), unaries->cols()));

#if 1
    print_grid(*m_unaries,"unaries");
    print_grid(*m_marginals,"marginals");
    print_grid(*m_newMarginals,"newMarginals");
#endif

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
  static void print_grid(const ProbabilitiesGrid& grid, const std::string& variableName = "")
  {
    std::cout << variableName << "\n";
    for(int i = 0; i < grid.rows(); ++i)
    {
      for(int j = 0; j < grid.cols(); ++j)
      {
        std::cout << "(" << i << "," << j << ")" << tvgutil::make_limited_container(grid(i,j),5) << "\n";
      }
    }
    std::cout << std::endl;
  }

  /**
   * \brief Runs a single mean-field update step on the CRF.
   */
  void update()
  {
    for(size_t row = 0, rows = m_marginals->rows(); row < rows; ++row)
    {
      for(size_t col = 0, cols = m_marginals->cols(); col < cols; ++col)
      {
        const std::map<Label,float>& phi_i = (*m_unaries)(row, col);
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

        float oneOverZ_i = 1.0f / Z_i;
        std::map<Label,float>& Q_i = (*m_newMarginals)(row, col);
        for(typename std::map<Label,float>::const_iterator it = M_i.begin(), iend = M_i.end(); it != iend; ++it)
        {
          const Label& L = it->first;
          float M_i_L = it->second;
          Q_i[L] = oneOverZ_i * exp(-M_i_L);
        }
      }
    }

    ++m_time;
    std::swap(m_marginals, m_newMarginals);
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  float compute_M_i_L(size_t row, size_t col, const Label& L, float phi_i_L) const
  {
    float result = phi_i_L;

    const std::map<Label,float>& phi_i = (*m_unaries)(row, col);
    for(typename std::map<Label,float>::const_iterator it = phi_i.begin(), iend = phi_i.end(); it != iend; ++it)
    {
      const Label& LDash = it->first;
      for(std::vector<Eigen::Vector2i>::const_iterator jt = m_neighbourOffsets.begin(), jend = m_neighbourOffsets.end(); jt != jend; ++jt)
      {
        // TODO: Check the ranges.
        const std::map<Label,float>& Q_j = (*m_marginals)(row + jt->y(), col + jt->x());
        float Q_j_LDash = Q_j.find(LDash)->second;
        float phi_i_j_L_LDash = compute_pairwise(L, LDash);
        result += Q_j_LDash * phi_i_j_L_LDash;
      }
    }

    return result;
  }

  float compute_pairwise(const Label& L, const Label& LDash) const
  {
    if(L == LDash)
    {
      return 0.0f;
    }
    else
    {
      // TODO
      return 0.0f;
    }
  }

  int get_patch_col(int globalCol, int OffsetIndex)
  {
    int col = globalCol + m_neighbourOffsets[OffsetIndex].x();
    if(col < 0)
    {
      col = 0;
    }
    else if(col > (m_width - 1))
    {
      col = m_width - 1;
    }
    return col;
  }

  int get_patch_row(int globalRow, int OffsetIndex)
  {
    int row = globalRow + m_neighbourOffsets[OffsetIndex].y();
    if(row < 0)
    {
      row = 0;
    }
    else if(row > (m_height - 1))
    {
      row = m_height - 1;
    }
    return row;
  }

  float M_per_node_label(int row, int col, Label l)
  {
    float M_unary = (*m_unaries)(row, col).find(l)->second;
    float M_pairwise = 0.0f;

    for(size_t i = 0, size = m_neighbourOffsets.size(); i < size; ++i)
    {
      typename std::map<Label,float>::const_iterator lPrimeIt, lPrimeItend;
      std::map<Label,float> unary = (*m_unaries)(0,0);
      for(lPrimeIt = unary.begin(), lPrimeItend = unary.end(); lPrimeIt != lPrimeItend; ++lPrimeIt)
      {
        std::cout << "get_patch_row: " << get_patch_row(row,i) << "\n";
        std::cout << "get_patch_col: " << get_patch_col(col,i) << "\n";
        M_pairwise += (*m_marginals)(get_patch_row(row, i), get_patch_col(col, i)).find(lPrimeIt->first)->second *
        get_pairwise(l, lPrimeIt->first);
      }
      
    }

    return M_unary + M_pairwise;
  }
};

}

#endif
