/**
 * rafl: RandomForest.h
 */

#ifndef H_RAFL_RANDOMFOREST
#define H_RAFL_RANDOMFOREST

#include <vector>

#include <tvgutil/SharedPtr.h>

#include "base/Example.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a reservoir that can be used to store examples in a random forest node.
 */
template <typename Label>
class ExampleReservoir
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The examples in the reservoir. */
  std::vector<Example> m_examples;

  /** The maximum number of examples allowed in the reservoir at any one time. */
  size_t m_maxSize;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Constructs a reservoir that can store at most the specified number of examples.
   *
   * Adding more than the specified number of examples to the reservoir will result in some
   * of the older examples being (randomly) discarded.
   *
   * \param maxSize The maximum number of examples allowed in the reservoir at any one time.
   */
  explicit ExampleReservoir(size_t maxSize)
  : m_maxSize(maxSize)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds an example to the reservoir.
   *
   * If the reservoir is currently full, an older example will be (randomly) discarded to
   * make space for the new example.
   *
   * \param example The example to be added.
   */
  void add_example(const Example& example)
  {
    // TODO
  }
};


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
   * \param examples  The examples to be added.
   */
  void add_examples(const std::vector<Example>& examples)
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
  /** The decision trees that collectively make up the random forest. */
  std::vector<Tree_Ptr> m_trees;

  // TODO

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds new training examples to the forest.
   *
   * \param examples  The examples to be added.
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
