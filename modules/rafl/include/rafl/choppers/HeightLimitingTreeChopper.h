/**
 * rafl: HeightLimitingTreeChopper.h
 */

#ifndef H_RAFL_HEIGHTLIMITINGTREECHOPPER
#define H_RAFL_HEIGHTLIMITINGTREECHOPPER

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops the first tree it finds whose height (strictly) exceeds a specified threshold.
 */
template <typename Label>
class HeightLimitingTreeChopper : public TreeChopper<Label>
{
  //#################### USINGS #################### 
private:
  using TreeChopper<Label>::RF_CPtr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The maximum height a tree may have before it becomes liable to be chopped. */
  size_t m_maxTreeHeight;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a height-limiting tree chopper.
   *
   * \param maxTreeHeight The maximum height a tree may have before it becomes liable to be chopped.
   */
  explicit HeightLimitingTreeChopper(size_t maxTreeHeight)
  : m_maxTreeHeight(maxTreeHeight)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> choose_tree_to_chop(const RF_CPtr& forest) const
  {
    for(size_t i = 0, treeCount = forest->get_tree_count(); i < treeCount; ++i)
    {
      if(forest->get_tree_depth(i) > m_maxTreeHeight) return i;
    }
    return boost::none;
  }
};

}

#endif
