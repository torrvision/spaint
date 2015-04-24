/**
 * rafl: DecisionTree.h
 */

#ifndef H_RAFL_DECISIONTREE
#define H_RAFL_DECISIONTREE

#include <set>
#include <stdexcept>

#include <tvgutil/PriorityQueue.h>
#include <tvgutil/PropertyUtil.h>

#include "decisionfunctions/DecisionFunctionGeneratorFactory.h"
#include "examples/ExampleReservoir.h"
#include "examples/ExampleUtil.h"

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

    /** The depth of the node in the tree. */
    size_t m_depth;

    /** The index of the node's left child in the tree's node array. */
    int m_leftChildIndex;

    /** The reservoir of examples currently stored in the node. */
    ExampleReservoir<Label> m_reservoir;

    /** The index of the node's right child in the tree's node array. */
    int m_rightChildIndex;

    /** The split function for the node. */
    DecisionFunction_Ptr m_splitter;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~

    /**
     * \brief Constructs a node.
     *
     * \param depth                 The depth of the node in the tree.
     * \param maxClassSize          The maximum number of examples of each class allowed in the node's reservoir at any one time.
     * \param randomNumberGenerator A random number generator.
     */
    Node(size_t depth, size_t maxClassSize, const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator)
    : m_depth(depth), m_leftChildIndex(-1), m_reservoir(maxClassSize, randomNumberGenerator), m_rightChildIndex(-1)
    {}
  };

public:
  /**
   * \brief An instance of this class can be used to provide the settings needed to configure a decision tree.
   */
  class Settings
  {
    //~~~~~~~~~~~~~~~~~~~~ TYPEDEFS ~~~~~~~~~~~~~~~~~~~~
  private:
    typedef boost::shared_ptr<const DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_CPtr;

    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~
  public:
    /** The number of different decision functions to consider when splitting a node. */
    int candidateCount;

    /** A generator that can be used to pick appropriate decision functions for nodes. */
    DecisionFunctionGenerator_CPtr decisionFunctionGenerator;

    /** The minimum information gain required for a node split to be acceptable. */
    float gainThreshold;

    /** The maximum number of examples of each class allowed in a node's reservoir at any one time. */
    size_t maxClassSize;

    /** The maximum height allowed for a tree. */
    size_t maxTreeHeight;

    /** A random number generator. */
    tvgutil::RandomNumberGenerator_Ptr randomNumberGenerator;

    /** The minimum number of examples that must have been added to an example reservoir before its containing node can be split. */
    size_t seenExamplesThreshold;

    /** A threshold splittability below which nodes should not be split (must be > 0). */
    float splittabilityThreshold;

    /** Whether or not to enable PMF reweighting to better handle a class imbalance in the training data. */
    bool usePMFReweighting;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~
  public:
    /**
     * \brief Default constructor.
     */
    Settings()
    {}

    /**
     * \brief Attempts to load settings from the specified XML file.
     *
     * This will throw if the settings cannot be successfully loaded.
     *
     * \param filename The name of the file.
     */
    explicit Settings(const std::string& filename)
    {
      using tvgutil::PropertyUtil;
      boost::property_tree::ptree tree = PropertyUtil::load_properties_from_xml(filename);
      initialise(PropertyUtil::make_property_map(tree));
    }

    /**
     * \brief Loads settings from a property map.
     *
     * \param properties  The property map.
     */
    explicit Settings(const std::map<std::string,std::string>& properties)
    {
      initialise(properties);
    }

    //~~~~~~~~~~~~~~~~~~~~ PRIVATE MEMBER FUCNTIONS ~~~~~~~~~~~~~~~~~~~~
  private:
    /**
     * \brief Loads settings from a property map.
     *
     * \param properties  The property map.
     */
    void initialise(const std::map<std::string,std::string>& properties)
    {
      std::string decisionFunctionGeneratorType;
      unsigned int randomSeed = 0;

      #define GET_SETTING(param) tvgutil::MapUtil::typed_lookup(properties, #param, param);
        GET_SETTING(candidateCount);
        GET_SETTING(decisionFunctionGeneratorType);
        GET_SETTING(gainThreshold);
        GET_SETTING(maxClassSize);
        GET_SETTING(maxTreeHeight);
        GET_SETTING(randomSeed);
        GET_SETTING(seenExamplesThreshold);
        GET_SETTING(splittabilityThreshold);
        GET_SETTING(usePMFReweighting);
      #undef GET_SETTING

      randomNumberGenerator.reset(new tvgutil::RandomNumberGenerator(randomSeed));
      decisionFunctionGenerator = DecisionFunctionGeneratorFactory<Label>::instance().make(decisionFunctionGeneratorType, randomNumberGenerator);
    }
  };

  //#################### PUBLIC TYPEDEFS ####################
public:
  typedef boost::shared_ptr<const DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_CPtr;

  //#################### PRIVATE TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef boost::shared_ptr<Node> Node_Ptr;
  typedef tvgutil::PriorityQueue<int,float,signed char,std::greater<float> > SplittabilityQueue;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The histogram holding the class frequencies observed in the training data. */
  Histogram<Label> m_classFrequencies;

  /** The indices of nodes to which examples have been added during the current call to add_examples() and whose splittability may need recalculating. */
  std::set<int> m_dirtyNodes;

  /** The inverses of the L1-normalised class frequencies observed in the training data. */
  boost::optional<std::map<Label,float> > m_inverseClassWeights;

  /** The nodes in the tree. */
  std::vector<Node_Ptr> m_nodes;

  /** The root node's index in the node array. */
  int m_rootIndex;

  /** The settings needed to configure the decision tree. */
  Settings m_settings;

  /** A priority queue of nodes that ranks them by how suitable they are for splitting. */
  SplittabilityQueue m_splittabilityQueue;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an empty decision tree.
   *
   * \param settings  The settings needed to configure the decision tree.
   */
  explicit DecisionTree(const Settings& settings)
  : m_settings(settings)
  {
    m_rootIndex = add_node(0);
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
    // Create a vector of indices indicating that all the examples should be added to the tree.
    size_t size = examples.size();
    std::vector<size_t> indices(size);
    for(size_t i = 0; i < size; ++i) indices[i] = i;

    add_examples(examples, indices);
  }

  /**
   * \brief Adds new training examples to the decision tree.
   *
   * \param examples                      A pool of examples that could potentially be added.
   * \param indices                       The indices of the examples in the pool that should be added to the decision tree.
   * \throws std::out_of_range_exception  If any of the indices are invalid.
   */
  void add_examples(const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices)
  {
    // Add each example indicated in the indices list to the tree.
    for(size_t i = 0, size = indices.size(); i < size; ++i)
    {
      add_example(examples.at(indices[i]));
    }

    // Update the inverse class weights (note that this must be done before updating the dirty nodes,
    // since the splittability calculations for the dirty nodes depend on the new weights).
    update_inverse_class_weights();

    // Recalculate the splittabilities of nodes to which examples have been added.
    update_dirty_nodes();
  }

  /**
   * \brief Looks up the probability mass function for the leaf to which an example with the specified descriptor would be added.
   *
   * \param descriptor  The descriptor.
   * \return            The probability mass function for the leaf to which an example with that descriptor would be added.
   */
  ProbabilityMassFunction<Label> lookup_pmf(const Descriptor_CPtr& descriptor) const
  {
    int leafIndex = find_leaf(*descriptor);
    return make_pmf(leafIndex);
  }

  /**
   * \brief Outputs the decision tree to a stream.
   *
   * \param os  The stream to which to output the tree.
   */
  void output(std::ostream& os) const
  {
    output_subtree(os, m_rootIndex, "");
  }

  /**
   * \brief Predicts a label for the specified descriptor.
   *
   * \param descriptor  The descriptor.
   * \return            The predicted label.
   */
  Label predict(const Descriptor_CPtr& descriptor) const
  {
    return lookup_pmf(descriptor).calculate_best_label();
  }

  /**
   * \brief Trains the tree by splitting a number of suitable nodes.
   *
   * The number of nodes that are split in each training step is limited to ensure that a step is not overly costly.
   *
   * \param splitBudget The maximum number of nodes that may be split in this training step.
   */
  void train(size_t splitBudget)
  {
    size_t nodesSplit = 0;

    // Keep splitting nodes until we either run out of nodes to split or exceed the split budget. In practice,
    // we will also stop splitting if the best node's splittability falls below a threshold. If the best node
    // cannot be split at present, we remove it from the queue to give the other nodes a chance and re-add it
    // at the end of the training step.
    std::vector<typename SplittabilityQueue::Element> elementsToReAdd;
    while(!m_splittabilityQueue.empty() && nodesSplit < splitBudget)
    {
      typename SplittabilityQueue::Element e = m_splittabilityQueue.top();
      if(e.key() >= m_settings.splittabilityThreshold)
      {
        m_splittabilityQueue.pop();
        if(split_node(e.id())) ++nodesSplit;
        else elementsToReAdd.push_back(e);
      }
      else break;
    }

    // Re-add any elements corresponding to nodes that could not be successfully split in this training step.
    for(typename std::vector<typename SplittabilityQueue::Element>::iterator it = elementsToReAdd.begin(), iend = elementsToReAdd.end(); it != iend; ++it)
    {
      m_splittabilityQueue.insert(it->id(), it->key(), it->data());
    }
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
    int leafIndex = find_leaf(*example->get_descriptor());

    // Add the example to the leaf's reservoir.
    m_nodes[leafIndex]->m_reservoir.add_example(example);

    // Mark the leaf as dirty to ensure that its splittability is properly recalculated once all of the examples have been added.
    m_dirtyNodes.insert(leafIndex);

    // Update the class frequency histogram.
    m_classFrequencies.add(example->get_label());
  }

  /**
   * \brief Adds a node to the decision tree.
   *
   * \param depth The depth of the node in the tree.
   *
   * \return The ID of the newly-added node.
   */
  int add_node(size_t depth)
  {
    m_nodes.push_back(Node_Ptr(new Node(depth, m_settings.maxClassSize, m_settings.randomNumberGenerator)));
    int id = static_cast<int>(m_nodes.size()) - 1;
    const signed char nullData = -1;
    m_splittabilityQueue.insert(id, 0.0f, nullData);
    return id;
  }

  /**
   * \brief Fills the specified reservoir with examples sampled from an input set of examples.
   *
   * \param inputExamples The set of examples from which to sample.
   * \param multipliers   The per-class ratios between the total number of examples seen for a class and the number of examples currently in the source reservoir.
   * \param reservoir     The reservoir to fill.
   */
  void fill_reservoir(const std::vector<Example_CPtr>& inputExamples, const std::map<Label,float>& multipliers, ExampleReservoir<Label>& reservoir)
  {
    // Group the input examples by label.
    std::map<Label,std::vector<Example_CPtr> > inputExamplesByLabel;
    for(typename std::vector<Example_CPtr>::const_iterator it = inputExamples.begin(), iend = inputExamples.end(); it != iend; ++it)
    {
      inputExamplesByLabel[(*it)->get_label()].push_back(*it);
    }

    // For each group:
    for(typename std::map<Label,std::vector<Example_CPtr> >::const_iterator it = inputExamplesByLabel.begin(), iend = inputExamplesByLabel.end(); it != iend; ++it)
    {
#if 1
      // Sample the appropriate number of examples (based on the multiplier for the group) and add them to the target reservoir.
      typename std::map<Label,float>::const_iterator jt = multipliers.find(it->first);
      if(jt == multipliers.end()) throw std::runtime_error("The input examples appear to be from a different reservoir than the multipliers");

      float multiplier = jt->second;
      size_t sampleCount = static_cast<size_t>(it->second.size() * multiplier + 0.5f);
      std::vector<Example_CPtr> sampledExamples = sample_examples(it->second, sampleCount);
      for(size_t j = 0; j < sampleCount; ++j)
      {
        reservoir.add_example(sampledExamples[j]);
      }
#else
      // Simply add all of the examples for the group to the target reservoir (useful for debugging purposes).
      for(size_t j = 0, size = it->second.size(); j < size; ++j)
      {
        reservoir.add_example(it->second[j]);
      }
#endif
    }
  }

  /**
   * \brief Finds the index of the leaf to which an example with the specified descriptor would currently be added.
   *
   * \param descriptor  The descriptor.
   * \return            The index of the leaf to which an example with the descriptor would currently be added.
   */
  int find_leaf(const Descriptor& descriptor) const
  {
    int curIndex = m_rootIndex;
    while(!is_leaf(curIndex))
    {
      curIndex = m_nodes[curIndex]->m_splitter->classify_descriptor(descriptor) == DecisionFunction::DC_LEFT ? m_nodes[curIndex]->m_leftChildIndex : m_nodes[curIndex]->m_rightChildIndex;
    }
    return curIndex;
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
   * \brief Makes a probability mass function for the specified leaf.
   *
   * \param leafIndex The leaf for which to make the probability mass function.
   * \return          The probability mass function.
   */
  ProbabilityMassFunction<Label> make_pmf(int leafIndex) const
  {
    return ProbabilityMassFunction<Label>(*m_nodes[leafIndex]->m_reservoir.get_histogram(), m_inverseClassWeights);
  }

  /**
   * \brief Outputs a subtree of the decision tree to a stream.
   *
   * \param os                The stream to which to output the subtree.
   * \param subtreeRootIndex  The index of the node at the root of the subtree.
   * \param indent            An indentation string to use in order to offset the output of the subtree.
   */
  void output_subtree(std::ostream& os, int subtreeRootIndex, const std::string& indent) const
  {
    int leftChildIndex = m_nodes[subtreeRootIndex]->m_leftChildIndex;
    int rightChildIndex = m_nodes[subtreeRootIndex]->m_rightChildIndex;
    DecisionFunction_Ptr splitter = m_nodes[subtreeRootIndex]->m_splitter;

    // Output the current node.
    os << indent << subtreeRootIndex << ": ";
    if(splitter) os << *splitter;
    else os << m_nodes[subtreeRootIndex]->m_reservoir.seen_examples() << ' ' << make_pmf(subtreeRootIndex);
    os << '\n';

    // Recursively output any children of the current node.
    if(leftChildIndex != -1) output_subtree(os, leftChildIndex, indent + "  ");
    if(rightChildIndex != -1) output_subtree(os, rightChildIndex, indent + "  ");
  }

  /**
   * \brief Randomly samples sampleCount examples (with replacement) from the specified set of input examples.
   *
   * \param inputExamples The set of examples from which to sample.
   * \param sampleCount   The number of samples to choose.
   * \return              The chosen set of examples.
   */
  std::vector<Example_CPtr> sample_examples(const std::vector<Example_CPtr>& inputExamples, size_t sampleCount)
  {
    std::vector<Example_CPtr> outputExamples;
    for(size_t i = 0; i < sampleCount; ++i)
    {
      int exampleIndex = m_settings.randomNumberGenerator->generate_int_from_uniform(0, static_cast<int>(inputExamples.size()) - 1);
      outputExamples.push_back(inputExamples[exampleIndex]);
    }
    return outputExamples;
  }

  /**
   * \brief Attempts to split the node with the specified index.
   *
   * \param nodeIndex The index of the node to try and split.
   * \return          true, if the node was successfully split, or false otherwise.
   */
  bool split_node(int nodeIndex)
  {
    Node& n = *m_nodes[nodeIndex];
    typename DecisionFunctionGenerator<Label>::Split_CPtr split = m_settings.decisionFunctionGenerator->split_examples(n.m_reservoir, m_settings.candidateCount, m_settings.gainThreshold, m_inverseClassWeights);
    if(!split) return false;

    // Set the decision function of the node to be split.
    n.m_splitter = split->m_decisionFunction;

    // Add left and right child nodes and populate their example reservoirs based on the chosen split.
    size_t childDepth = n.m_depth + 1;
    n.m_leftChildIndex = add_node(childDepth);
    n.m_rightChildIndex = add_node(childDepth);
    std::map<Label,float> multipliers = n.m_reservoir.get_class_multipliers();
    fill_reservoir(split->m_leftExamples, multipliers, m_nodes[n.m_leftChildIndex]->m_reservoir);
    fill_reservoir(split->m_rightExamples, multipliers, m_nodes[n.m_rightChildIndex]->m_reservoir);

    // Update the splittability for the child nodes.
    update_splittability(n.m_leftChildIndex);
    update_splittability(n.m_rightChildIndex);

    // Clear the example reservoir in the node that was split.
    n.m_reservoir.clear();

    return true;
  }

  /**
   * \brief Updates the splittability values for any nodes whose reservoirs were changed whilst adding exmaples.
   */
  void update_dirty_nodes()
  {
    for(std::set<int>::const_iterator it = m_dirtyNodes.begin(), iend = m_dirtyNodes.end(); it != iend; ++it)
    {
      update_splittability(*it);
    }

    // Clear the list of dirty nodes once their splittability has been updated.
    m_dirtyNodes.clear();
  }

  /**
   * \brief Updates the splittability of the specified node.
   *
   * \param nodeIndex  The index of the node.
   */
  void update_splittability(int nodeIndex)
  {
    // Recalculate the node's splittability.
    const ExampleReservoir<Label>& reservoir = m_nodes[nodeIndex]->m_reservoir;
    float splittability;
    if(m_nodes[nodeIndex]->m_depth + 1 < m_settings.maxTreeHeight && reservoir.seen_examples() >= m_settings.seenExamplesThreshold)
    {
      splittability = ExampleUtil::calculate_entropy(*reservoir.get_histogram(), m_inverseClassWeights);
    }
    else
    {
      splittability = 0.0f;
    }

    // Update the splittability queue to reflect the node's new splittability.
    m_splittabilityQueue.update_key(nodeIndex, splittability);
  }

  /**
   * \brief Updates the inverse class weights.
   */
  void update_inverse_class_weights()
  {
    if(!m_settings.usePMFReweighting) return;

    float count = static_cast<float>(m_classFrequencies.get_count());

    const std::map<Label,size_t>& bins = m_classFrequencies.get_bins();
    for(typename std::map<Label,size_t>::const_iterator it = bins.begin(), iend = bins.end(); it != iend; ++it)
    {
      (*m_inverseClassWeights)[it->first] = count / it->second;
    }
  }
};

}

#endif
