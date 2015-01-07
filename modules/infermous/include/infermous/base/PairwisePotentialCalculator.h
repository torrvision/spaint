/**
 * infermous: PairwisePotentialCalculator.h
 */

#ifndef H_INFERMOUS_PAIRWISEPOTENTIALCALCULATOR
#define H_INFERMOUS_PAIRWISEPOTENTIALCALCULATOR

#include <boost/shared_ptr.hpp>

namespace infermous {

/**
 * \brief An instance of a class derived from an instantiation of this class template can be used to calculate pairwise potentials for a 2D CRF.
 */
template <typename Label>
class PairwisePotentialCalculator
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the pairwise potential calculator.
   */
  virtual ~PairwisePotentialCalculator() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the pairwise potential between two nodes in the CRF based on their labels.
   *
   * Note that this should be symmetric, i.e. calculate_potential(l1,l2) should equal calculate_potential(l2,l1).
   *
   * \param l1  The label of one of the nodes.
   * \param l2  The label of the other node.
   * \return    The calculated pairwise potential between the two nodes.
   */
  virtual float calculate_potential(const Label& l1, const Label& l2) const = 0;
};

//#################### TYPEDEFS ####################

template <typename Label> using PairwisePotentialCalculator_Ptr = boost::shared_ptr<PairwisePotentialCalculator<Label> >;
template <typename Label> using PairwisePotentialCalculator_CPtr = boost::shared_ptr<const PairwisePotentialCalculator<Label> >;

}

#endif
