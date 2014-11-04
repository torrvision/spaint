/**
 * rafl: RandomForest.h
 */

#ifndef H_RAFL_RANDOMFOREST
#define H_RAFL_RANDOMFOREST

#include <map>
#include <vector>

#include <tvgutil/IDAllocator.h>
#include <tvgutil/SharedPtr.h>

#include "examples/Example.h"

namespace rafl {

template <typename Label>
class Node
{
  // TODO
};

/**
 * \brief An instance of an instantiation of this class template represents a decision tree suitable for use within a random forest.
 */
template <typename Label>
class DecisionTree
{
  //#################### PRIVATE VARIABLES ####################
private:
  // TODO

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds new training examples to the decision tree.
   *
   * \param examples  The IDs of the examples to be added.
   */
  void add_examples(const std::vector<int>& exampleIDs)
  {
    // TODO
  }
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
  /** The global example buffer. */
  tvgutil::shared_ptr<std::map<int,Example<Label> > > m_exampleBuffer;

  /** The ID allocator that is used to generate example IDs. */
  tvgutil::IDAllocator m_exampleIDAllocator;

  /** The decision trees that collectively make up the random forest. */
  std::vector<Tree_Ptr> m_trees;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds new training examples to the forest.
   *
   * \param examples  The examples to be added.
   */
  void add_examples(const std::vector<Example<Label> >& examples)
  {
    // Generate IDs for the new examples and add them to the global example buffer.
    std::vector<int> exampleIDs;
    exampleIDs.reserve(examples.size());
    for(typename std::vector<Example<Label> >::const_iterator it = examples.begin(), iend = examples.end(); it != iend; ++it)
    {
      int id = m_exampleIDAllocator.allocate();
      exampleIDs.push_back(id);
      m_exampleBuffer.insert(std::make_pair(id, *it));
    }

    // Add the new examples to the different trees.
    for(typename std::vector<Tree_Ptr>::const_iterator it = m_trees.begin(), iend = m_trees.end(); it != iend; ++it)
    {
      (*it)->add_examples(exampleIDs);
    }
  }
};

}

#endif
