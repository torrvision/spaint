/**
 * rafl: RandomTreeChopper.cpp
 */

#include "choppers/RandomTreeChopper.h"

namespace rafl {


//#################### CONSTRUCTORS ####################
RandomTreeChopper::RandomTreeChopper(size_t treeCount, size_t period, unsigned int seed)
: TreeChopper(treeCount, period), m_rng(seed)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################
boost::optional<size_t> RandomTreeChopper::calculate_tree_to_chop() const
{
  boost::optional<size_t> treeToChop;
  if(m_time++ % m_period == 0)
  {
    treeToChop.reset((m_rng.generate_int_from_uniform(0, static_cast<int>(m_treeCount) - 1)));
  }
  return treeToChop;
}

}
