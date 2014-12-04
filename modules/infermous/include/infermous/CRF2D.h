/**
 * infermous: CRF2D.h
 */

#ifndef H_INFERMOUS_CRF2D
#define H_INFERMOUS_CRF2D

#include <map>
#include <vector>

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

  //#################### CONSTRUCTORS ####################
public:
  CRF2D(const ProbabilitiesGrid_Ptr& unaries, int neighbourRadius)
  : m_time(0), m_unaries(unaries)
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
    for(int row = 0, rows = m_marginals->rows(); row < rows; ++row)
      for(int col = 0, cols = m_marginals->cols(); col < cols; ++col)
      {
        float Z_i = 0.0f;

        // TODO
        for(size_t i = 0, size = m_neighbourOffsets.size(); i < size; ++i)
        {
          
        }
      }
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  // TODO
};

}

#endif
