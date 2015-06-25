/**
 * rafl: TreeChopper.h
 */

#ifndef H_RAFL_TREECHOPPER
#define H_RAFL_TREECHOPPER

#include <boost/optional.hpp>

namespace rafl {

/**
 * \brief An instance of a class derived from this one represents a tree chopping strategy that can be used to chop trees in the forest based on various criteria.
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
   * \brief TODO.
   */
  virtual boost::optional<size_t> calculate_tree_to_chop() const = 0;
};

}

#endif
