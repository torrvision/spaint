/**
 * rafl: CyclicTreeChopper.h
 */

#ifndef H_RAFL_CYCLICTREECHOPPER
#define H_RAFL_CYCLICTREECHOPPER

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees in a cycle.
 */
template <typename Label>
class CyclicTreeChopper : public TreeChopper<Label>
{
  //#################### TYPEDEFS #################### 
private:
  typedef TreeChopper<Label> TC;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The number of three that have been chopped. */
  mutable size_t m_chopCount;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a cyclic tree chopper.
   *
   * \param treeCount The number of trees in the random forest.
   * \param period    The time period between successive chops.
   */
  CyclicTreeChopper(size_t treeCount, size_t period)
  : TC(treeCount, period), m_chopCount(0)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop(const typename TC::RF_Ptr& randomForest) const
  {
    boost::optional<size_t> treeToChop;
    if(this->time_to_chop())
    {
      treeToChop.reset(m_chopCount++ % this->m_treeCount);
    }
    return treeToChop;
  }
};

}

#endif
