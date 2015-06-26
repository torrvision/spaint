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

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The time period between successive chops. */
  size_t m_period;

  /** A count of the number of times the lunberjack has come to manage the trees. */
  mutable size_t m_time;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the tree chopper.
   */
  virtual ~TreeChopper() {}

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a cyclic chopper.
   *
   * \param treeCount The number of trees in the random forest.
   * \param period    The time period between successive chops.
   */
  TreeChopper(size_t period)
  : m_period(period), m_time(0)
  {}

  //#################### PUBLICABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the Id of a tree to be chopped, or boost::none if no tree needs chopping.
   */
  virtual boost::optional<size_t> calculate_tree_to_chop(const RF_Ptr& randomForest) const = 0;

  //#################### PROTECTED MEMBER FUNCTIONS #################### 
protected:
  /**
   * \brief Calculates whether its time to chop a tree.
   */
  bool time_to_chop() const
  {
    return m_time++ % m_period == 0;
  }
};

}

#endif
