/**
 * rafl: MaxHeightTreeChopper.h
 */

#ifndef H_RAFL_MAXHEIGHTTREECHOPPER
#define H_RAFL_MAXHEIGHTTREECHOPPER

#include <boost/shared_ptr.hpp>

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees based on their height.
 */
template <typename Label>
class MaxHeightTreeChopper : public TreeChopper<Label>
{
  //#################### TYPEDEFS #################### 
private:
  typedef TreeChopper<Label> TC;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The maximum height trees in the random forest may take before they are chopped. */
  size_t m_maxTreeHeight;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a maximum tree height chopper.
   *
   * \param maxTreeHeight The maximum height of a tree in the random forest before it gets chopped.
   */
  MaxHeightTreeChopper(size_t maxTreeHeight)
  : m_maxTreeHeight(maxTreeHeight)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop(const typename TC::RF_CPtr& randomForest) const
  {
    boost::optional<size_t> treeToChop;
    for(size_t i = 0, treeCount = randomForest->get_tree_count(); i < treeCount; ++i)
    {
      if(randomForest->get_tree_depth(i) > m_maxTreeHeight)
      {
        treeToChop.reset(i);
        break;
      }
    }
    return treeToChop;
  }
};

}

#endif
