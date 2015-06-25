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

  // #################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a cyclic tree chopper.
   *
   * \param treeCount The number of trees in the random forest.
   * \param period    The time period between successive chops.
   */
  CyclicTreeChopper(size_t treeCount, size_t period);

  // #################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop() const;
};

}

#endif
