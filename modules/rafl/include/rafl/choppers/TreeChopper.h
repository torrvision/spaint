/**
 * rafl: TreeChopper.h
 */

#ifndef H_RAFL_TREECHOPPER
#define H_RAFL_TREECHOPPER

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
  typedef boost::shared_ptr<RandomForest<Label> > RF_Ptr;
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
   * \brief Optionally chooses a tree in the random forest for chopping.
   *
   * \param forest  The random forest.
   * \return        The index of the tree to be chopped, if any, or boost::none otherwise.
   */
  virtual boost::optional<size_t> choose_tree_to_chop(const RF_CPtr& forest) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Chops an individual tree in the forest as necessary.
   *
   * Individual tree choppers can define "necessary" as they see fit.
   *
   * \param forest  The random forest.
   */
  void chop_tree_if_necessary(const RF_Ptr& forest) const
  {
    boost::optional<size_t> treeToChop = choose_tree_to_chop(forest);
    if(treeToChop) forest->reset_tree(*treeToChop);
  }
};

}

#endif
