/**
 * rafl: TreeChopper.h
 */

#ifndef H_RAFL_TREECHOPPER
#define H_RAFL_TREECHOPPER

#include <boost/optional.hpp>

namespace rafl {

/**
 * \brief An instance of a class derived from this one represents a tree chopping strategy for a random forest.
 */
class TreeChopper
{
  //#################### DESTRUCTOR #################### 
public:
  /**
   * \brief Destroys the tree chopper.
   */
  virtual ~TreeChopper() {}

  //#################### PUBLICABSTRACT MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief Calculates the Id of a tree to be chopped, or boost::none if no tree needs chopping.
   */
  virtual boost::optional<size_t> calculate_tree_to_chop() const = 0;
};

}

#endif
