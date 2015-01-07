/**
 * infermous: CRF2D.h
 */

#ifndef H_INFERMOUS_CRF2D
#define H_INFERMOUS_CRF2D

#include <map>
#include <ostream>

#include "CRFUtil.h"
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
  typedef infermous::PairwisePotentialCalculator_CPtr<Label> PairwisePotentialCalculator_CPtr;
  typedef infermous::ProbabilitiesGrid<Label> ProbabilitiesGrid;
  typedef infermous::ProbabilitiesGrid_Ptr<Label> ProbabilitiesGrid_Ptr;
  typedef infermous::ProbabilitiesGrid_CPtr<Label> ProbabilitiesGrid_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The height of the CRF. */
  int m_height;

  /** The grid of marginal probabilities that will be updated at each time step. */
  ProbabilitiesGrid_Ptr m_marginals;

  /** The pairwise potential calculator. */
  PairwisePotentialCalculator_CPtr m_pairwisePotentialCalculator;

  /** The grid of unary probabilities. */
  ProbabilitiesGrid_Ptr m_unaries;

  /** The width of the CRF. */
  int m_width;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a 2D CRF.
   *
   * \param unaries                     The grid of unary probabilities.
   * \param pairwisePotentialCalculator The pairwise potential calculator.
   */
  CRF2D(const ProbabilitiesGrid_Ptr& unaries, const PairwisePotentialCalculator_CPtr& pairwisePotentialCalculator)
  : m_height(static_cast<int>(unaries->rows())),
    m_pairwisePotentialCalculator(pairwisePotentialCalculator),
    m_unaries(unaries),
    m_width(static_cast<int>(unaries->cols()))
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
   * \brief Gets the marginal probabilities for the specified location.
   *
   * \param loc The location whose marginal probabilities we want to get.
   * \return    The marginal probabilities for the specified location.
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
   * \brief Gets the unary probabilities for the specified location.
   *
   * \param loc The location whose unary probabilities we want to get.
   * \return    The unary probabilities for the specified location.
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
   * \brief Predicts the labels for each pixel in the CRF.
   *
   * return The grid of predicted labels.
   */
  Grid<Label> predict_labels() const
  {
    return CRFUtil::predict_labels(*m_marginals);
  }

  /**
   * \brief Swaps the current grid of marginal probabilities with a new grid.
   *
   * This is useful for implementing a "double-buffering" update approach in which we update a new grid and then swap it with the old one at each time step.
   *
   * \param marginals The new grid of marginal probabilities.
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

  //#################### STREAM OPERATORS ####################

  /**
   * \brief Outputs a 2D CRF to the specified stream.
   *
   * \param os  The stream.
   * \param rhs The 2D CRF.
   * \return    The stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const CRF2D& rhs)
  {
    os << *rhs.m_marginals;
    return os;
  }
};

//#################### TYPEDEFS ####################

template <typename Label> using CRF2D_Ptr = boost::shared_ptr<CRF2D<Label> >;
template <typename Label> using CRF2D_CPtr = boost::shared_ptr<const CRF2D<Label> >;

}

#endif
