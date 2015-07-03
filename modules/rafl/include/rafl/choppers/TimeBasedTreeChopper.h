/**
 * rafl: TimeBasedTreeChopper.h
 */

#ifndef H_RAFL_TIMEBASEDTREECHOPPER
#define H_RAFL_TIMEBASEDTREECHOPPER

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that periodically applies another tree chopper.
 *
 * For computer scientists: This is an example of the Decorator pattern.
 */
template <typename Label>
class TimeBasedTreeChopper : public TreeChopper<Label>
{
  //#################### TYPEDEFS AND USINGS #################### 
private:
  using TreeChopper<Label>::RF_CPtr;
  typedef boost::shared_ptr<TreeChopper<Label> > TreeChopper_CPtr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** A nested chopper that will be applied periodically. */
  TreeChopper_CPtr m_nestedChopper;

  /** The time period between successive chops (every period'th attempt to chop will be forwarded to the nested chopper). */
  const size_t m_period;

  /** The number of times the lumberjack has come to manage the trees. */
  mutable size_t m_time;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a time-based tree chopper.
   *
   * \param nestedChopper A nested chopper that will be applied periodically.
   * \param period        The time period between successive chops (every period'th attempt to chop will be forwarded to the nested chopper).
   */
  TimeBasedTreeChopper(const TreeChopper_CPtr& nestedChopper, size_t period)
  : m_nestedChopper(nestedChopper), m_period(period), m_time(0)
  {}

//#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> choose_tree_to_chop(const RF_CPtr& forest) const
  {
    return m_time++ % m_period == 0 ? boost::optional<size_t>(m_nestedChopper->choose_tree_to_chop(forest)) : boost::none;
  }
};

}

#endif
