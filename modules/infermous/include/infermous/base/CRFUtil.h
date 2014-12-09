/**
 * infermous: CRFUtil.h
 */

#ifndef H_INFERMOUS_CRFUTIL
#define H_INFERMOUS_CRFUTIL

#include <iostream>

#include <Eigen/Dense>

#include <tvgutil/ArgUtil.h>
#include <tvgutil/LimitedContainer.h>

namespace infermous {

//#################### TYPEDEFS ####################

template <typename T> using Grid = Eigen::Matrix<T,-1,-1>;
template <typename Label> using ProbabilitiesGrid = Grid<std::map<Label,float> >;

/**
 * \brief This class contains various CRF utility functions.
 */
struct CRFUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Predicts the labels for each pixel in a grid of potentials by choosing a label with the highest potential for each pixel.
   *
   * \param potentials  The grid of potentials for whose pixels we want to predict labels.
   * return             A grid of predicted labels.
   */
  template <typename Label>
  static Grid<Label> predict_labels(const ProbabilitiesGrid<Label>& potentials)
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
 * \brief Outputs a probabilities grid to the specified stream.
 *
 * \param os  The stream.
 * \param rhs The grid.
 * \return    The stream.
 */
template <typename Label>
std::ostream& operator<<(std::ostream& os, const ProbabilitiesGrid<Label>& rhs)
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
