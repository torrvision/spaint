/**
 * rafl: CyclicTreeChopper.h
 */

#ifndef H_RAFL_CYCLICTREECHOPPER
#define H_RAFL_CYCLICTREECHOPPER

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees one after another in a cycle.
 */
template <typename Label>
class CyclicTreeChopper : public TreeChopper<Label>
{
  //#################### USINGS #################### 
private:
  using TreeChopper<Label>::RF_CPtr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The index of the next tree to chop. */
  mutable size_t m_nextTreeToChop;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a cyclic tree chopper.
   */
  CyclicTreeChopper()
  : m_nextTreeToChop(0)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> choose_tree_to_chop(const RF_CPtr& forest) const
  {
    size_t result = m_nextTreeToChop;
    m_nextTreeToChop = (m_nextTreeToChop + 1) % forest->get_tree_count();
    return result;
  }
};

}

#endif
