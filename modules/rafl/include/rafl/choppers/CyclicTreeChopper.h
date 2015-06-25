/**
 * rafl: CyclicTreeChopper.h
 */

#ifndef H_RAFL_CYCLICTREECHOPPER
#define H_RAFL_CYCLICTREECHOPPER

#include "../core/RandomForest.h"

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees in a cyclic order.
 */
class CyclicTreeChopper : public TreeChopper
{
  //#################### PRIVATE VARIABLES #################### 
private:
  /** The number of three that have been chopped. */
  mutable size_t m_chopCount;

  /** TODO. */
  size_t m_period;

  /** A count of the number of times the lunberjack has come to manage the trees. */
  mutable size_t m_time;

  /** TODO */
  size_t m_treeCount;

  // #################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a cyclic tree chopper.
   *
   * \param period The number of time steps to wait before chopping the next tree.
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
