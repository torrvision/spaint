/**
 * rafl: RandomForest.h
 */

#ifndef H_RAFL_RANDOMFOREST
#define H_RAFL_RANDOMFOREST

#include "DecisionTree.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a random forest.
 */
template <typename Label>
class RandomForest
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef DecisionTree<Label> DT;
  typedef boost::shared_ptr<DT> DT_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The decision trees that collectively make up the random forest. */
  std::vector<DT_Ptr> m_trees;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random forest.
   *
   * \param treeCount The number of decision trees to use in the random forest.
   * \param settings  The settings needed to configure the decision trees.
   */
  RandomForest(size_t treeCount, const typename DT::Settings& settings)
  {
    for(size_t i = 0; i < treeCount; ++i)
    {
      m_trees.push_back(DT_Ptr(new DT(settings)));
    }
  }

private:
  /**
   * \brief Constructs a random forest.
   *
   * Note: This constructor is needed for serialization and should not be used otherwise.
   */
  RandomForest() {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds new training examples to the forest.
   *
   * \param examples  The examples to be added.
   */
  void add_examples(const std::vector<Example_CPtr>& examples)
  {
    // Add the new examples to the different trees.
    for(typename std::vector<DT_Ptr>::const_iterator it = m_trees.begin(), iend = m_trees.end(); it != iend; ++it)
    {
      (*it)->add_examples(examples);
    }
  }

  /**
   * \brief Adds new training examples to the forest.
   *
   * \param examples                      A pool of examples that could potentially be added.
   * \param indices                       The indices of the examples in the pool that should be added to the forest.
   * \throws std::out_of_range_exception  If any of the indices are invalid.
   */
  void add_examples(const std::vector<Example_CPtr>& examples, const std::vector<size_t>& indices)
  {
    // Add the new examples to the different trees.
    for(typename std::vector<DT_Ptr>::const_iterator it = m_trees.begin(), iend = m_trees.end(); it != iend; ++it)
    {
      (*it)->add_examples(examples, indices);
    }
  }

  /**
   * \brief Calculate the average leaf entropy of a specified tree.
   *
   * \param treeId  The Id of the specified tree.
   * \return        The average leaf entropy of the specified tree.
   */
  float calculate_average_leaf_entropy(size_t treeId) const
  {
    return m_trees[treeId]->calculate_average_leaf_entropy();
  }

  /**
   * \brief Calculates an overall forest PMF for the specified descriptor.
   *
   * This is simply the average of the PMFs for the specified descriptor in the various decision trees.
   *
   * \param descriptor  The descriptor.
   * \return            The PMF.
   */
  ProbabilityMassFunction<Label> calculate_pmf(const Descriptor_CPtr& descriptor) const
  {
    // Sum the masses from the individual tree PMFs for the descriptor.
    std::map<Label,float> masses;
    for(typename std::vector<DT_Ptr>::const_iterator it = m_trees.begin(), iend = m_trees.end(); it != iend; ++it)
    {
      ProbabilityMassFunction<Label> individualPMF = (*it)->lookup_pmf(descriptor);
      const std::map<Label,float>& individualMasses = individualPMF.get_masses();
      for(typename std::map<Label,float>::const_iterator jt = individualMasses.begin(), jend = individualMasses.end(); jt != jend; ++jt)
      {
        masses[jt->first] += jt->second;
      }
    }

    // Create a normalised probability mass function from the summed masses.
    return ProbabilityMassFunction<Label>(masses);
  }

  /**
   * \brief Gets a histogram holding the class frequencies observed in the training data for a particular tree.
   *
   * \param treeId  The Id of the specified tree.
   * \return        A histogram holding the class frequencies observed in the training data for a specified tree.
   */
  const Histogram<Label>& get_class_frequencies(size_t treeId) const
  {
    return m_trees[treeId]->get_class_frequencies();
  }

  /**
   * \brief Gets the number of nodes in a specified tree.
   *
   * \param treeId  The Id of the specified tree.
   * \return        The number of nodes in the specified tree.
   */
  size_t get_node_count(size_t treeId) const
  {
    return m_trees[treeId]->get_node_count();
  }

  /**
   * \brief Gets the number of trees in the random forest.
   *
   * \reutrn The number of trees in the random forest.
   */
  size_t get_tree_count() const
  {
    return m_trees.size();
  }

  /**
   * \brief Gets the depth of the specified tree.
   *
   * \return  The depth of the tree.
   */
  size_t get_tree_depth(size_t treeId) const
  {
    return m_trees[treeId]->get_tree_depth();
  }
  
  /**
   * \brief Gets whether or not the forest is valid.
   *
   * Forests are invalid until we have started training them.
   *
   * \return  true, if the forest is valid, or false otherwise.
   */
  bool is_valid() const
  {
    for(typename std::vector<DT_Ptr>::const_iterator it = m_trees.begin(), iend = m_trees.end(); it != iend; ++it)
    {
      if(!(*it)->is_valid()) return false;
    }
    return true;
  }

  /**
   * \brief Outputs the random forest to a stream.
   *
   * \param os  The stream to which to output the forest.
   */
  void output(std::ostream& os) const
  {
    for(size_t i = 0, size = m_trees.size(); i < size; ++i)
    {
      os << "Tree " << i << ":\n";
      m_trees[i]->output(os);
      os << '\n';
    } }

  /**
   * \brief Outputs statistics about the random forest to a stream.
   *
   * \param os  The stream to which to output the statistics.
   */
  void output_statistics(std::ostream& os) const
  {
    os << std::setprecision(5);
    for(size_t i = 0, size = m_trees.size(); i < size; ++i)
    {
      os << "Tree: " << i << ", ";
      os << "Node Count: " << get_node_count(i) << ", ";
      os << "Depth: " << get_tree_depth(i) << ", ";
      os << "Avg. Leaf Entropy: " << calculate_average_leaf_entropy(i) << ", ";
      os << "Class Frequencies: " << get_class_frequencies(i) << '\n';
    }
    os << '\n';
  }

  /**
   * \brief Predicts a label for the specified descriptor.
   *
   * \param descriptor  The descriptor.
   * \return            The predicted label.
   */
  Label predict(const Descriptor_CPtr& descriptor) const
  {
    return calculate_pmf(descriptor).calculate_best_label();
  }

  /**
   * \brief Resets a specified tree in the random forest with specific settings.
   */
  void chop_tree(const boost::optional<size_t>& treeId, const typename DT::Settings& settings)
  {
    if(treeId)
    {
      if(*treeId < m_trees.size())
      {
        m_trees[*treeId] = DT_Ptr(new DT(settings));
      }
    }
  }

  /**
   * \brief Trains the forest by splitting a number of suitable nodes in each tree.
   *
   * The number of nodes that are split in each training step is limited to ensure that a step is not overly costly.
   *
   * \param splitBudget The maximum number of nodes per tree that may be split in this training step.
   * \return            The total number of nodes that have been split across all the trees.
   */
  size_t train(size_t splitBudget)
  {
    size_t nodesSplit = 0;
    for(typename std::vector<DT_Ptr>::const_iterator it = m_trees.begin(), iend = m_trees.end(); it != iend; ++it)
    {
      nodesSplit += (*it)->train(splitBudget);
    }
    return nodesSplit;
  }

  //#################### SERIALIZATION ####################
private:
  /**
   * \brief Serializes the random forest to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & m_trees;
  }

  friend class boost::serialization::access;
};

}

#endif
