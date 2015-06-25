/**
 * rafl: CyclicTreeChopper.h
 */

#ifndef H_RAFL_CYCLICTREECHOPPER
#define H_RAFL_CYCLICTREECHOPPER

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees in a cyclic order.
 */
class CyclicTreeChopper : public TreeChopper
{
  //#################### PRIVATE VARIABLES #################### 
private:
  /** TODO. */
  size_t m_period;

  // #################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a cyclic tree chopper.
   *
   * \param period The number of time steps to wait before chopping the next tree.
   */
  CyclicTreeChopper(size_t period)
  : m_period(period)
  {}

  // #################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual size_t calculate_tree_to_chop() const
  {
    //TODO
    return 0;
  }
};

}

#endif
