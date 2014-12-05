/**
 * infermous: CRF2D.h
 */

#ifndef H_INFERMOUS_CRF2D
#define H_INFERMOUS_CRF2D

#include <map>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

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
  : m_height(unaries->rows()), m_pairwisePotentialCalculator(pairwisePotentialCalculator), m_timeStep(0), m_unaries(unaries), m_width(unaries->cols())
  {
    m_marginals.reset(new ProbabilitiesGrid(*unaries));
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
   * \param row TODO
   * \param col TODO
   * \return    TODO
   */
  const std::map<Label,float>& get_marginals(int row, int col) const
  {
    return (*m_marginals)(row, col);
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
   * \param row TODO
   * \param col TODO
   * \return    TODO
   */
  const std::map<Label,float>& get_unaries(int row, int col) const
  {
    return (*m_unaries)(row, col);
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
  void set_marginals(const ProbabilitiesGrid_Ptr& marginals)
  {
    m_marginals = marginals;
  }

  /**
   * \brief Determines whether or not the specified (row,col) location is within the bounds of the CRF.
   *
   * \param row The row.
   * \param col The column.
   * \return    true, if the specified location is within the bounds of the CRF, or false otherwise.
   */
  bool within_bounds(int row, int col) const
  {
    // TODO
    throw 23;
  }
};

}

#endif
