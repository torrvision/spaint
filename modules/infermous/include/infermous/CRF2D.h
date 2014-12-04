/**
 * infermous: CRF2D.h
 */

#ifndef H_INFERMOUS_CRF2D
#define H_INFERMOUS_CRF2D

#include <map>
#include <vector>
#include <cmath>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

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
  void update()
  {
    typename std::map<Label,float>::const_iterator lIt, lItend;
    std::map<Label,float> unary = (*m_unaries)(0,0);
    for(lIt = unary.begin(), lItend = unary.end(); lIt != lItend; ++lIt)
    {
      update_per_label(lIt->first);
    }
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  void update_per_label(Label l)
  {
    for(int row = 0, rows = m_marginals->rows(); row < rows; ++row)
      for(int col = 0, cols = m_marginals->cols(); col < cols; ++col)
      {
        std::map<Label,float> M_i;
        float Z_i = 0.0f;

        typename std::map<Label,float>::const_iterator lIt, lItend;
        std::map<Label,float> unary = (*m_unaries)(0,0);
        for(lIt = unary.begin(), lItend = unary.end(); lIt != lItend; ++lIt)
        {
          M_i.insert(std::make_pair(lIt->first, M_per_node_label(row, col, lIt->first)));
          Z_i += exp( M_i.find(lIt->first)->second );
        }

        (*m_newMarginals)(row, col)[l] = (1/Z_i)*exp( - M_i.find(l)->second );
      }
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
        M_pairwise += (*m_marginals)(get_patch_row(row, i), get_patch_col(col, i)).find(lPrimeIt->first)->second *
        get_pairwise(l, lPrimeIt->first);
      }
      
    }

    return M_unary + M_pairwise;
  }

  int get_patch_row(int globalRow, int OffsetIndex)
  {
    int row = globalRow + m_neighbourOffsets[OffsetIndex].x();
    if(row < 0)
    {
      return 0;
    }
    else if(row > m_height)
    {
      return m_height;
    }
    else
    {
      return row;
    }
  }

  int get_patch_col(int globalCol, int OffsetIndex)
  {
    int col = globalCol + m_neighbourOffsets[OffsetIndex].y();
    if(col < 0)
    {
      return 0;
    }
    else if(col > m_width)
    {
      return m_width;
    }
    else{
      return col;
    }
  }

  float calculate_pairwise()
  {
    //TODO;
    return 0;
  }

  float get_pairwise(Label l, Label l_prime)
  {
    return l == l_prime ? 0 : calculate_pairwise();
  }

  };

}

#endif
