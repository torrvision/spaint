/**
 * rafl: RandomTreeChopper.h
 */

#ifndef H_RAFL_RANDOMTREECHOPPER
#define H_RAFL_RANDOMTREECHOPPER

#include <tvgutil/RandomNumberGenerator.h>

#include "TreeChopper.h"

namespace rafl {

/**
 * \brief An instance of this class represents a tree chopper that chops trees at random.
 */
class RandomTreeChopper : public TreeChopper
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The random number generator to use when chopping trees. */
  mutable tvgutil::RandomNumberGenerator m_rng;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random tree chopper.
   *
   * \param treeCount The number of trees in the random forest.
   * \param period    The time period between successive chops.
   * \param seed      The seed for the random number generator.
   */
  RandomTreeChopper(size_t treeCount, size_t period, unsigned int seed);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual boost::optional<size_t> calculate_tree_to_chop() const;
};

}

#endif
