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
 * For computer scientists: this is an example of the Decorator pattern.
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
  /** The most recent iteration of the lumberjack's chopping cycle. */
  mutable size_t m_iteration;

  /** A nested chopper that will be applied periodically. */
  TreeChopper_CPtr m_nestedChopper;

  /** The time period between successive chops (every period'th attempt to chop will be forwarded to the nested chopper). */
  const size_t m_period;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a time-based tree chopper.
   *
   * \param nestedChopper A nested chopper that will be applied periodically.
   * \param period        The time period between successive chops (every period'th attempt to chop will be forwarded to the nested chopper).
   */
  TimeBasedTreeChopper(const TreeChopper_CPtr& nestedChopper, size_t period)
  : m_iteration(period - 1), m_nestedChopper(nestedChopper), m_period(period)
  {}

//#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> choose_tree_to_chop(const RF_CPtr& forest) const
  {
    m_iteration = (m_iteration + 1) % m_period;
    return m_iteration == 0 ? boost::optional<size_t>(m_nestedChopper->choose_tree_to_chop(forest)) : boost::none;
  }
};

}

#endif
