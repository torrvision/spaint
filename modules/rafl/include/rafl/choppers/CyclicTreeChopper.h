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
  //#################### TYPEDEFS #################### 
private:
  typedef TreeChopper<Label> TC;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The number of trees that have been chopped. */
  mutable size_t m_chopCount;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a cyclic tree chopper.
   */
  CyclicTreeChopper()
  : m_chopCount(0)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> try_select_tree_to_chop(const typename TC::RF_CPtr& forest) const
  {
    boost::optional<size_t> treeToChop;
    treeToChop.reset(m_chopCount++ % forest->get_tree_count());
    return treeToChop;
  }
};

}

#endif
