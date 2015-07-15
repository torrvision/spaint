/**
 * rafl: HeightLimitingTreeChopper.h
 */

#ifndef H_RAFL_HEIGHTLIMITINGTREECHOPPER
#define H_RAFL_HEIGHTLIMITINGTREECHOPPER

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops a randomly selected tree if its height (strictly) exceeds a specified threshold.
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
    int randomTree = m_rng.generate_int_from_uniform(0, static_cast<int>(forest->get_tree_count()) - 1);
    if(forest->get_tree(randomTree)->get_tree_depth() > m_maxTreeHeight)
    {
      return randomTree;
    }
    else
    {
      return boost::none;
    }
  }
};

}

#endif
