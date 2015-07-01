/**
 * rafl: TimeBasedTreeChopper.h
 */

#ifndef H_RAFL_TIMEBASEDTREECHOPPER
#define H_RAFL_TIMEBASEDTREECHOPPER

#include <boost/shared_ptr.hpp>

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a time based tree chopper that indicates whether to chop a tree or not based on an internal clock.
 */
template <typename Label>
class TimeBasedTreeChopper : public TreeChopper<Label>
{
  //#################### TYPEDEFS #################### 
private:
  typedef TreeChopper<Label> TC;
  typedef boost::shared_ptr<const TC> TC_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The time period between successive chops. */
  size_t m_period;

  /** An internal counter to record the number of times the lumberjack has come to manage the trees. */
  mutable size_t m_time;

  /** A tree chopper. */
  TC_CPtr m_treeChopper;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a time-based tree chopper.
   *
   * \param period    The time period between successive chops.
   */
  TimeBasedTreeChopper(const TC_CPtr& treeChopper, size_t period)
  : m_period(period), m_time(0), m_treeChopper(treeChopper)
  {}

//#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop(const typename TC::RF_Ptr& randomForest) const
  {
    boost::optional<size_t> treeToChop;
    if(m_time++ % m_period == 0)
    {
      treeToChop = m_treeChopper->calculate_tree_to_chop(randomForest);
    }
    return treeToChop;
  }
};

}

#endif
