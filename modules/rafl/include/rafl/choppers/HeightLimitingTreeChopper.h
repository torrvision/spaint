/**
 * rafl: HeightLimitingTreeChopper.h
 */

#ifndef H_RAFL_HEIGHTLIMITINGTREECHOPPER
#define H_RAFL_HEIGHTLIMITINGTREECHOPPER

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that finds all trees whose height (strictly) exceeds a specified threshold and randomly chops one.
 *
 * If there are no trees of sufficient height, the chopper leaves the forest unchanged.
 */
template <typename Label>
class HeightLimitingTreeChopper : public TreeChopper<Label>
{
  //#################### USINGS #################### 
private:
  using typename TreeChopper<Label>::RF_CPtr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The maximum height a tree may have before it becomes liable to be chopped. */
  size_t m_maxTreeHeight;

  /** The random number generator to use when deciding which tree to chop. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief Constructs a height-limiting tree chopper.
   *
   * \param maxTreeHeight The maximum height a tree may have before it becomes liable to be chopped.
   */
  explicit HeightLimitingTreeChopper(size_t maxTreeHeight, unsigned int seed)
  : m_maxTreeHeight(maxTreeHeight), m_rng(seed)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual boost::optional<size_t> choose_tree_to_chop(const RF_CPtr& forest) const
  {
    // Find all trees whose height exceeds the threshold.
    std::vector<size_t> tallTrees;
    for(size_t i = 0, count = forest->get_tree_count(); i < count; ++i)
    {
      if(forest->get_tree(i)->get_tree_depth() > m_maxTreeHeight)
      {
        tallTrees.push_back(i);
      }
    }

    if(tallTrees.empty())
    {
      // If there are no trees of sufficient height, leave the forest unchanged.
      return boost::none;
    }
    else
    {
      // Otherwise, randomly pick a tree of sufficient height for chopping.
      return tallTrees[m_rng.generate_int_from_uniform(0, static_cast<int>(tallTrees.size()) - 1)];
    }
  }
};

}

#endif
