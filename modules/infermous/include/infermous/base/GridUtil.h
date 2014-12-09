/**
 * infermous: GridUtil.h
 */

#ifndef H_INFERMOUS_GRIDUTIL
#define H_INFERMOUS_GRIDUTIL

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <tvgutil/ArgUtil.h>
#include <tvgutil/LimitedContainer.h>

namespace infermous {

/** \brief An instance of an instantiation of this type can be used to represent a 2D grid of elements. */
template <typename T> using Grid = Eigen::Matrix<T,-1,-1>;

/** \brief An instance of an instantiation of this type can be used to represent a 2D grid of label -> potential maps. */
template <typename Label> using PotentialsGrid = Grid<std::map<Label,float> >;

template <typename T> using Grid_Ptr = boost::shared_ptr<Grid<T> >;
template <typename T> using Grid_CPtr = boost::shared_ptr<const Grid<T> >;
template <typename Label> using PotentialsGrid_Ptr = boost::shared_ptr<PotentialsGrid<Label> >;
template <typename Label> using PotentialsGrid_CPtr = boost::shared_ptr<const PotentialsGrid<Label> >;

/**
 * \brief This class contains various CRF utility functions.
 */
struct GridUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Predicts the labels for each pixel in a potentials grid by choosing a label with the highest potential for each pixel.
   *
   * \param grid  The grid of potentials for whose pixels we want to predict labels.
   * return       The grid of predicted labels.
   */
  template <typename Label>
  static Grid<Label> predict_labels(const PotentialsGrid<Label>& potentials)
  {
    Grid<Label> result(potentials.cols(), potentials.rows());
    for(size_t y = 0, height = potentials.rows(); y < height; ++y)
    {
      for(size_t x = 0, width = potentials.cols(); x < width; ++x)
      {
        result(x, y) = tvgutil::ArgUtil::argmax(potentials(x, y));
      }
    }
    return result;
  }
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a grid to the specified stream.
 *
 * \param os  The stream.
 * \param rhs The grid.
 * \return    The stream.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Grid<T>& rhs)
{
  for(size_t y = 0, height = rhs.rows(); y < height; ++y)
  {
    for(size_t x = 0, width = rhs.cols(); x < width; ++x)
    {
      os << rhs(x, y) << ' ';
    }
    os << '\n';
  }
  return os;
}

/**
 * \brief Outputs a potentials grid to the specified stream.
 *
 * \param os  The stream.
 * \param rhs The grid.
 * \return    The stream.
 */
template <typename Label>
std::ostream& operator<<(std::ostream& os, const PotentialsGrid<Label>& rhs)
{
  for(size_t y = 0, height = rhs.rows(); y < height; ++y)
  {
    for(size_t x = 0, width = rhs.cols(); x < width; ++x)
    {
      os << '(' << x << ',' << y << ") " << tvgutil::make_limited_container(rhs(x, y), 5) << '\n';
    }
  }
  return os;
}

}

#endif
