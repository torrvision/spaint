/**
 * rafl: RandomTreeChopper.h
 */

#ifndef H_RAFL_RANDOMTREECHOPPER
#define H_RAFL_RANDOMTREECHOPPER

#include <tvgutil/RandomNumberGenerator.h>

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees in the forest at random.
 */
template <typename Label>
class RandomTreeChopper : public TreeChopper<Label>
{
  //#################### TYPEDEFS #################### 
private:
  typedef TreeChopper<Label> TC;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The random number generator to use when deciding which tree to chop. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random tree chopper.
   *
   * \param seed    The seed for the random number generator.
   */
  RandomTreeChopper(unsigned int seed)
  : m_rng(seed)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop(const typename TC::RF_CPtr& randomForest) const
  {
    boost::optional<size_t> treeToChop;
    treeToChop.reset((m_rng.generate_int_from_uniform(0, static_cast<int>(randomForest->get_tree_count()) - 1)));
    return treeToChop;
  }
};

}

#endif
