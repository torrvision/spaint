/**
 * rafl: DecisionTree.h
 */

#ifndef H_RAFL_DECISIONTREE
#define H_RAFL_DECISIONTREE

#include <functional>
#include <map>
#include <stdexcept>

#include <tvgutil/PriorityQueue.h>

#include "base/DecisionFunction.h"
#include "examples/Example.h"
#include "examples/ExampleReservoir.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a tree suitable for use within a random forest.
 */
template <typename Label>
class DecisionTree
{
  //#################### NESTED TYPES ####################
private:
  /**
   * \brief An instance of this struct represents a node in the tree.
   */
  struct Node
  {
    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~

    /** The index of the node's left child. */
    int m_leftChildIndex;

    /** The reservoir of examples currently stored in the node. */
    ExampleReservoir<Label> m_reservoir;

    /** The index of the node's right child. */
    int m_rightChildIndex;

    /** The split function for the node. */
    DecisionFunction_Ptr m_splitter;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~

    /**
     * \brief Constructs a node.
     *
     * \param maxReservoirSize  The maximum number of examples that can be stored in the node's reservoir.
     * \param rng               A random number generator.
     */
    Node(size_t maxReservoirSize, const tvgutil::RandomNumberGenerator_Ptr& rng)
    : m_leftChildIndex(-1), m_reservoir(maxReservoirSize, rng), m_rightChildIndex(-1)
    {}
  };

  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef boost::shared_ptr<Node> Node_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The indices of nodes to which examples have been added during the current call to add_examples() and whose splittability may need recalculating. */
  std::vector<int> m_dirtyNodes;

  /** The nodes in the tree. */
  std::vector<Node_Ptr> m_nodes;

  /** The root node's index in the node array. */
  int m_rootIndex;

  /** A priority queue of nodes that ranks them by how suitable they are for splitting. */
  tvgutil::PriorityQueue<int,float,void*,std::greater<float> > m_splittabilityQueue;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an empty decision tree.
   *
   * \param maxReservoirSize  The maximum number of examples that can be stored in a node's reservoir.
   * \param rng               A random number generator.
   */
  explicit DecisionTree(size_t maxReservoirSize, const tvgutil::RandomNumberGenerator_Ptr& rng)
  : m_rootIndex(0)
  {
    m_nodes.push_back(Node_Ptr(new Node(maxReservoirSize, rng)));
    m_splittabilityQueue.insert(m_rootIndex, 0.0f, NULL);
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds new training examples to the decision tree.
   *
   * \param examples  The examples to be added.
   */
  void add_examples(const std::vector<Example_CPtr>& examples)
  {
    // Add each example to the tree.
    for(std::vector<int>::const_iterator it = examples.begin(), iend = examples.end(); it != iend; ++it)
    {
      add_example(*it);
    }

    // Update the splittability values for any nodes whose reservoirs were changed whilst adding examples.
    for(std::vector<int>::const_iterator it = m_dirtyNodes.begin(), iend = m_dirtyNodes.end(); it != iend; ++it)
    {
      update_splittability(*it);
    }

    // Clear the list of dirty nodes once their splittability has been updated.
    std::vector<int>().swap(m_dirtyNodes);
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Adds a new training example to the decision tree.
   *
   * \param example The example to be added.
   */
  void add_example(const Example_CPtr& example)
  {
    // Find the leaf to which to add the new example.
    const Descriptor& descriptor = example->get_descriptor();
    int curIndex = m_rootIndex;
    while(!is_leaf(curIndex))
    {
      curIndex = *m_nodes[curIndex]->m_splitter(descriptor) ? m_nodes[curIndex]->m_leftChildIndex : m_nodes[curIndex]->m_rightChildIndex;
    }

    // Add the example to the leaf's reservoir.
    if(m_nodes[curIndex]->m_reservoir.add_example(example))
    {
      // If the leaf's reservoir changed as a result of adding the example, record this fact to ensure that its splittability is properly recalculated.
      m_dirtyNodes.push_back(curIndex);
    }
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

  /**
   * \brief Updates the splittability of the specified node.
   *
   * \param nodeIndex  The index of the node.
   */
  void update_splittability(int nodeIndex)
  {
    // Recalculate the node's splittability.
    const ExampleReservoir& reservoir = m_nodes[nodeIndex]->m_reservoir;
    float splittability = static_cast<float>(reservoir.current_size()) / reservoir.max_size();

    // Update the splittability queue to reflect the node's new splittability.
    m_splittabilityQueue.update_key(nodeIndex, splittability);
  }
};

}

#endif
