/**
 * rafl: MaxHeightTreeChopper.h
 */

#ifndef H_RAFL_MAXHEIGHTTREECHOPPER
#define H_RAFL_MAXHEIGHTTREECHOPPER

#include <boost/shared_ptr.hpp>

#include "TreeChopper.h"

#include <iostream>

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees based on the tree heights.
 */
template <typename Label>
class MaxHeightTreeChopper : public TreeChopper<Label>
{
  //#################### TYPEDEFS #################### 
private:
  typedef boost::shared_ptr<RandomForest<Label> > RF_Ptr;
  typedef TreeChopper<Label> TC;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The maximum height of the trees in the random forest. */
  size_t m_maxTreeHeight;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a mas tree height chopper.
   *
   * \param randomForest  The random forest beign monitored for overgrowth.
   * \param maxTreeHeight The maximum height of a tree in the random forest before it gets chopped down.
   * \param period        The time period between successive chops.
   */
  MaxHeightTreeChopper(const typename TC::RF_Ptr& randomForest, size_t maxTreeHeight, size_t period)
  : TC(period), m_maxTreeHeight(maxTreeHeight)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop(const typename TC::RF_Ptr& randomForest) const
  {
    boost::optional<size_t> treeToChop;
    if(this->time_to_chop())
    {
      for(size_t i = 0, treeCount = randomForest->get_tree_count(); i < treeCount; ++i)
      {
        if(randomForest->get_tree_depth(i) > m_maxTreeHeight)
        {
          treeToChop.reset(i);
          break;
        }
      }
    }
    return treeToChop;
  }
};

}

#endif
