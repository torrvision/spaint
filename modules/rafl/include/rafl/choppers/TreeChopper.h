/**
 * rafl: TreeChopper.h
 */

#ifndef H_RAFL_TREECHOPPER
#define H_RAFL_TREECHOPPER

#include "../core/RandomForest.h"

#include <boost/optional.hpp>

namespace rafl {

/**
 * \brief An instance of a class derived from this one represents a tree chopping strategy for a random forest.
 */
template <typename Label>
class TreeChopper
{
  //#################### TYPEDEFS #################### 
protected:
  typedef boost::shared_ptr<RandomForest<Label> > RF_Ptr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the tree chopper.
   */
  virtual ~TreeChopper() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the Id of a tree to be chopped, or boost::none if no tree needs chopping.
   *
   * \param randomForest  The random forest being monitored.
   * \return              The (optional) Id of the tree to be chopped.
   */
  virtual boost::optional<size_t> calculate_tree_to_chop(const RF_Ptr& randomForest) const = 0;
};

}

#endif
