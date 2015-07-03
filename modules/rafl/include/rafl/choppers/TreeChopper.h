/**
 * rafl: TreeChopper.h
 */

#ifndef H_RAFL_TREECHOPPER
#define H_RAFL_TREECHOPPER

#include <boost/optional.hpp>

#include "../core/RandomForest.h"

namespace rafl {

/**
 * \brief An instance of a class derived from this one can be used to chop trees in a random forest.
 *
 * Tree chopping is used as a way of allowing a forest to "forget" things that it has previously learnt and that are now out-of-date.
 */
template <typename Label>
class TreeChopper
{
  //#################### TYPEDEFS #################### 
protected:
  typedef boost::shared_ptr<const RandomForest<Label> > RF_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the tree chopper.
   */
  virtual ~TreeChopper() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to select a tree in a random forest for chopping.
   *
   * \param forest  The random forest.
   * \return        The ID of the tree to be chopped, if any, or boost::none otherwise.
   */
  virtual boost::optional<size_t> try_select_tree_to_chop(const RF_CPtr& forest) const = 0;
};

}

#endif
