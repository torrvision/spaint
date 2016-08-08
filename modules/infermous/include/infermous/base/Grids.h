/**
 * infermous: Grids.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_INFERMOUS_GRIDS
#define H_INFERMOUS_GRIDS

#include <ostream>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <tvgutil/containers/LimitedContainer.h>

namespace infermous {

//#################### TYPEDEFS ####################

/** \brief An instance of an instantiation of this type can be used to represent a 2D grid of elements. */
template <typename T> using Grid = Eigen::Matrix<T,-1,-1>;

template <typename T> using Grid_Ptr = boost::shared_ptr<Grid<T> >;
template <typename T> using Grid_CPtr = boost::shared_ptr<const Grid<T> >;

/** \brief An instance of an instantiation of this type can be used to represent a 2D grid of label -> probability maps. */
template <typename Label> using ProbabilitiesGrid = Grid<std::map<Label,float> >;

template <typename Label> using ProbabilitiesGrid_Ptr = boost::shared_ptr<ProbabilitiesGrid<Label> >;
template <typename Label> using ProbabilitiesGrid_CPtr = boost::shared_ptr<const ProbabilitiesGrid<Label> >;

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
