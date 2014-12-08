/**
 * infermous: CRFUtil.h
 */

#ifndef H_INFERMOUS_CRFUTIL
#define H_INFERMOUS_CRFUTIL

#include <iostream>
#include <map>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <tvgutil/LimitedContainer.h>

namespace infermous {

struct CRFUtil
{
  //#################### TYPEDEFS ####################
  //typedef Eigen::Matrix<std::map<Label,float>,-1,-1> ProbabilitiesGrid;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
  /**
   * \brief TODO
   */
  template <typename Label>
  static void print_grid(std::ostream& os, const Eigen::Matrix<std::map<Label,float>,-1,-1>& grid)
  {
    for(int i = 0; i < grid.rows(); ++i)
    {
      for(int j = 0; j < grid.cols(); ++j)
      {
        os << "(" << i << "," << j << ") " << tvgutil::make_limited_container(grid(i,j),5) << "\n";
      }
    }
    os << std::endl;
  }

  /**
   * \brief TODO
   */
  template <typename Label>
  static Eigen::Matrix<Label,-1,-1> predict_labels(const Eigen::Matrix<std::map<Label,float>,-1,-1>& potentials)
  {
    Eigen::Matrix<Label,-1,-1> result(potentials.size());

    for(size_t y = 0, height = potentials.rows(); y < height; ++y)
    {
      for(size_t x = 0, width = potentials.cols(); x < width; ++x)
      {
        Label bestLabel;

      }
    }

    return result;
  }
};

}

#endif
