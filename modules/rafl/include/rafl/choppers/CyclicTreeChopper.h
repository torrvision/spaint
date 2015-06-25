/**
 * rafl: CyclicTreeChopper.h
 */

#ifndef H_RAFL_CYCLICTREECHOPPER
#define H_RAFL_CYCLICTREECHOPPER

#include "../core/RandomForest.h"

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees in a cycle.
 */
class CyclicTreeChopper : public TreeChopper
{
  //#################### PRIVATE VARIABLES #################### 
private:
  /** The number of three that have been chopped. */
  mutable size_t m_chopCount;

  /** The time period between successive chops. */
  size_t m_period;

  /** A count of the number of times the lunberjack has come to manage the trees. */
  mutable size_t m_time;

  /** The number of trees in the random forest. */
  size_t m_treeCount;

  // #################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a cyclic tree chopper.
   *
   * \param treeCount The number of trees in the random forest.
   * \param period    The time period between successive chops.
   */
  CyclicTreeChopper(size_t treeCount, size_t period)
  : m_chopCount(0), m_period(period), m_time(0), m_treeCount(treeCount)
  {}

  // #################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop() const
  {
    boost::optional<size_t> treeToChop;
    if(m_time++ % m_period == 0)
    {
      treeToChop.reset(m_chopCount++ % m_treeCount);
    }
    return treeToChop;
  }
};

}

#endif
