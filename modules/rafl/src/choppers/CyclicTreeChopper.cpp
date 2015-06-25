/**
 * rafl: CyclicTreeChopper.cpp
 */

#include "choppers/CyclicTreeChopper.h"

namespace rafl {

// #################### CONSTRUCTORS ####################

CyclicTreeChopper::CyclicTreeChopper(size_t treeCount, size_t period)
: TreeChopper(treeCount, period), m_chopCount(0)
{}

// #################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<size_t> CyclicTreeChopper::calculate_tree_to_chop() const
{
  boost::optional<size_t> treeToChop;
  if(m_time++ % m_period == 0)
  {
    treeToChop.reset(m_chopCount++ % m_treeCount);
  }
  return treeToChop;
}

}
