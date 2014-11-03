/**
 * rafl: RandomForest.h
 */

#ifndef H_RAFL_RANDOMFOREST
#define H_RAFL_RANDOMFOREST

#include <vector>

#include <tvgutil/SharedPtr.h>

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a decision tree suitable for use within a random forest.
 */
template <typename Label>
class DecisionTree
{

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public: 
  void add_examples(const std::vector<Example>& examples)
  {

  }
  // TODO
};

/**
 * \brief An instance of an instantiation of this class template represents a random forest.
 */
template <typename Label>
class RandomForest
{
  //#################### TYPEDEFS ####################
private:
  typedef tvgutil::shared_ptr<DecisionTree<Label> > Tree_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  std::vector<Tree_Ptr> m_trees;
  // TODO

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  void add_examples(const std::vector<Example>& examples)
  {
    for(size_t i = 0, size = m_trees.size(); i < size; ++i)
    {
      m_trees[i]->add_examples(examples);
    }
  }
};

}

#endif
