/**
 * rafl: RandomForest.h
 */

#ifndef H_RAFL_RANDOMFOREST
#define H_RAFL_RANDOMFOREST

#include <map>
#include <stdexcept>
#include <vector>

#include <tvgutil/IDAllocator.h>
#include <tvgutil/SharedPtr.h>

#include "examples/Example.h"
#include "examples/ExampleReservoir.h"

namespace rafl {

/**
 * \brief An instance of this class represents the decision function of a node in a random forest.
 *
 * Decision functions (as currently implemented) test whether an individual feature is less than a threshold.
 */
class DecisionFunction
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The index of the feature in a feature descriptor that should be compared to the threshold. */
  size_t m_featureIndex;

  /** The threshold against which to compare it. */
  float m_threshold;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a decision function.
   *
   * \param featureIndex  The index of the feature in the feature descriptor that should be compared to the threshold.
   * \param threshold     The threshold against which to compare it.
   */
  DecisionFunction(size_t featureIndex, float threshold)
  : m_featureIndex(featureIndex), m_threshold(threshold)
  {}

  //#################### PUBLIC OPERATORS ####################
public:
  /**
   * \brief Evaluates the decision function for the specified feature descriptor.
   *
   * \param descriptor  The feature descriptor for which to evaluate the decision function.
   * \return            true, if the feature being tested is less than the threshold, or false otherwise.
   */
  bool operator()(const Descriptor& descriptor) const
  {
    return descriptor[m_featureIndex] < m_threshold;
  }
};

typedef tvgutil::shared_ptr<DecisionFunction> DecisionFunction_Ptr;
  
/**
 * \brief An instance of an instantiation of this class template represents a tree suitable for use within a random forest.
 */
template <typename Label>
class DecisionTree
{
  //#################### NESTED TYPES ####################
private:
  /**
   * \brief An instance of this class represents a node in the tree.
   */
  struct Node
  {
    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~

    /** The index of the node's left child. */
    int m_leftChildIndex;

    /** The reservoir of examples currently stored in the node. */
    ExampleReservoir m_reservoir;

    /** The index of the node's right child. */
    int m_rightChildIndex;

    /** The split function for the node. */
    DecisionFunction_Ptr m_splitter;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~

    /**
     * \brief Constructs a node.
     */
    Node()
    : m_leftChildIndex(-1), m_rightChildIndex(-1)
    {}
  };

  //#################### TYPEDEFS ####################
private:
  typedef tvgutil::shared_ptr<Node> Node_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The global example buffer. */
  tvgutil::shared_ptr<std::map<int,Example<Label> > > m_exampleBuffer;

  /** The nodes in the tree. */
  std::vector<Node_Ptr> m_nodes;

  /** The root node's index in the node array. */
  int m_rootIndex;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an empty decision tree.
   *
   * \param exampleBuffer	The global example buffer.
   */
  explicit DecisionTree(const tvgutil::shared_ptr<std::map<int,Example<Label> > >& exampleBuffer)
  : m_exampleBuffer(exampleBuffer), m_rootIndex(0)
  {
    m_nodes.push_back(Node_Ptr(new Node));
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds new training examples to the decision tree.
   *
   * \param exampleIDs  The IDs of the examples to be added.
   */
  void add_examples(const std::vector<int>& exampleIDs)
  {
    for(std::vector<int>::const_iterator it = exampleIDs.begin(), iend = exampleIDs.end(); it != iend; ++it)
    {
      add_example(*it);
    }
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Adds a new training example to the decision tree.
   *
   * \param exampleID  The ID of the example to be added.
   */
  void add_example(int exampleID)
  {
    // Find the leaf to which to add the new example.
    typename std::map<int,Example<Label> >::const_iterator it = m_exampleBuffer->find(exampleID);
    if(it == m_exampleBuffer->end()) throw std::runtime_error("DecisionTree::add_example: Example ID not found");

    const Descriptor& descriptor = *it->second.get_descriptor();
    int curIndex = m_rootIndex;
    while(!is_leaf(curIndex))
    {
      curIndex = *m_nodes[curIndex]->m_splitter(descriptor) ? m_nodes[curIndex]->m_leftChildIndex : m_nodes[curIndex]->m_rightChildIndex;
    }

    // Add the example to the leaf's reservoir.
    m_nodes[curIndex]->m_reservoir.add_example(exampleID);
  }

  /**
   * \brief Returns whether or not the specified node is a leaf.
   *
   * \param nodeIndex  The index of the node.
   * \return           true, if the specified node is a leaf, or false otherwise.
   */
  bool is_leaf(int nodeIndex) const
  {
    return m_nodes[nodeIndex]->m_leftChildIndex == -1;
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
