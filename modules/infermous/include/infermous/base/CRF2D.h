/**
 * infermous: CRF2D.h
 */

#ifndef H_INFERMOUS_CRF2D
#define H_INFERMOUS_CRF2D

#include <iostream>
#include <map>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <tvgutil/LimitedContainer.h>

#include "GridUtil.h"
#include "PairwisePotentialCalculator.h"

namespace infermous {

/**
 * \brief An instance of an instantiation of this class template represents a 2D conditional random field.
 */
template <typename Label>
class CRF2D
{
  //#################### TYPEDEFS ####################
public:
  typedef PairwisePotentialCalculator_CPtr<Label> PairwisePotentialCalculator_CPtr;
  typedef PotentialsGrid<Label> PotentialsGrid;
  typedef PotentialsGrid_Ptr<Label> PotentialsGrid_Ptr;
  typedef PotentialsGrid_CPtr<Label> PotentialsGrid_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The height of the CRF. */
  int m_height;

  /** The grid of marginal potentials that will be updated at each time step. */
  PotentialsGrid_Ptr m_marginals;

  /** The pairwise potential calculator. */
  PairwisePotentialCalculator_CPtr m_pairwisePotentialCalculator;

  /** The current time step. */
  size_t m_timeStep;

  /** The grid of unary potentials. */
  PotentialsGrid_Ptr m_unaries;

  /** The width of the CRF. */
  int m_width;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a 2D CRF.
   *
   * \param unaries                     The grid of unary potentials.
   * \param pairwisePotentialCalculator The pairwise potential calculator.
   */
  CRF2D(const PotentialsGrid_Ptr& unaries, const PairwisePotentialCalculator_CPtr& pairwisePotentialCalculator)
  : m_height(static_cast<int>(unaries->rows())),
    m_pairwisePotentialCalculator(pairwisePotentialCalculator),
    m_timeStep(0),
    m_unaries(unaries),
    m_width(static_cast<int>(unaries->cols()))
  {
    m_marginals.reset(new PotentialsGrid(*unaries));
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the height of the CRF.
   *
   * \return  The height of the CRF.
   */
  int get_height() const
  {
    return m_height;
  }

  /**
   * \brief Gets the grid of marginal potentials.
   *
   * \return  The grid of marginal potentials.
   */
  PotentialsGrid_CPtr get_marginals() const
  {
    return m_marginals;
  }

  /**
   * \brief Gets the marginal potentials for the specified location.
   *
   * \param loc The location whose marginal potentials we want to get.
   * \return    The marginal potentials for the specified location.
   */
  const std::map<Label,float>& get_marginals_at(const Eigen::Vector2i& loc) const
  {
    return (*m_marginals)(loc.y(), loc.x());
  }

  /**
   * \brief Gets the pairwise potential calculator.
   *
   * \return  The pairwise potential calculator.
   */
  PairwisePotentialCalculator_CPtr get_pairwise_potential_calculator() const
  {
    return m_pairwisePotentialCalculator;
  }

  /**
   * \brief Gets the time step of the CRF.
   *
   * \return  The time step of the CRF.
   */
  size_t get_time_step() const
  {
    return m_timeStep;
  }

  /**
   * \brief Gets the unary potentials for the specified location.
   *
   * \param loc The location whose unary potentials we want to get.
   * \return    The unary potentials for the specified location.
   */
  const std::map<Label,float>& get_unaries_at(const Eigen::Vector2i& loc) const
  {
    return (*m_unaries)(loc.y(), loc.x());
  }

  /**
   * \brief Gets the width of the CRF.
   *
   * \return  The width of the CRF.
   */
  int get_width() const
  {
    return m_width;
  }

  /**
   * \brief Increments the time step of the CRF.
   */
  void increment_time_step()
  {
    ++m_timeStep;
  }

  /**
   * \brief Swaps the current grid of marginal potentials with a new grid.
   *
   * This is useful for implementing a "double-buffering" update approach in which we update a new grid and then swap it with the old one at each time step.
   *
   * \param marginals The new grid of marginal potentials.
   */
  void swap_marginals(PotentialsGrid_Ptr& marginals)
  {
    std::swap(m_marginals, marginals);
  }

  /**
   * \brief Determines whether or not the specified location is within the bounds of the CRF.
   *
   * \param loc The location.
   * \return    true, if the specified location is within the bounds of the CRF, or false otherwise.
   */
  bool within_bounds(const Eigen::Vector2i& loc) const
  {
    return 0 <= loc.x() && loc.x() < m_width && 0 <= loc.y() && loc.y() < m_height;
  }
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a 2D CRF to the specified stream.
 *
 * \param os  The stream.
 * \param rhs The 2D CRF.
 * \return    The stream.
 */
template <typename Label>
std::ostream& operator<<(std::ostream& os, const CRF2D<Label>& rhs)
{
  os << *rhs.get_marginals();
  return os;
}

//#################### TYPEDEFS ####################

template <typename Label> using CRF2D_Ptr = boost::shared_ptr<CRF2D<Label> >;
template <typename Label> using CRF2D_CPtr = boost::shared_ptr<const CRF2D<Label> >;

}

#endif
