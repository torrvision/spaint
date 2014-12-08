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
  typedef boost::shared_ptr<const PairwisePotentialCalculator<Label> > PairwisePotentialCalculator_CPtr;
  typedef Eigen::Matrix<std::map<Label,float>,-1,-1> ProbabilitiesGrid;
  typedef boost::shared_ptr<ProbabilitiesGrid> ProbabilitiesGrid_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The height of the CRF. */
  int m_height;

  /** TODO */
  ProbabilitiesGrid_Ptr m_marginals;

  /** The pairwise potential calculator. */
  PairwisePotentialCalculator_CPtr m_pairwisePotentialCalculator;

  /** The current time step. */
  size_t m_timeStep;

  /** TODO */
  ProbabilitiesGrid_Ptr m_unaries;

  /** The width of the CRF. */
  int m_width;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a 2D CRF.
   *
   * \param unaries                     TODO
   * \param pairwisePotentialCalculator The pairwise potential calculator.
   */
  CRF2D(const ProbabilitiesGrid_Ptr& unaries, const PairwisePotentialCalculator_CPtr& pairwisePotentialCalculator)
  : m_height(static_cast<int>(unaries->rows())),
    m_pairwisePotentialCalculator(pairwisePotentialCalculator),
    m_timeStep(0),
    m_unaries(unaries),
    m_width(static_cast<int>(unaries->cols()))
  {
    m_marginals.reset(new ProbabilitiesGrid(*unaries));
  }

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:

  void output(std::ostream& os) const
  {
    print_grid(os, *m_marginals);
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
   * \brief TODO
   *
   * \param loc TODO
   * \return    TODO
   */
  const std::map<Label,float>& get_marginals(const Eigen::Vector2i& loc) const
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
   * \brief TODO
   *
   * \param loc TODO
   * \return    TODO
   */
  const std::map<Label,float>& get_unaries(const Eigen::Vector2i& loc) const
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
   * \brief TODO
   *
   * \param marginals TODO
   */
  void swap_marginals(ProbabilitiesGrid_Ptr& marginals)
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

}

#endif
