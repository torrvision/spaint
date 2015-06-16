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
    }
  }

  /**
   * \brief Outputs the random forest statistics to a stream.
   *
   * \param os  The stream to which to output the statistics.
   */
  void output_statistics(std::ostream& os) const
  {
    os << std::setprecision(5);
    for(size_t i = 0, size = m_trees.size(); i < size; ++i)
    {
      os << "Tree: " << i << ", ";
      os << "NodeCount: " << m_trees[i]->get_node_count() << ", ";
      os << "Depth: " << m_trees[i]->get_tree_depth() << ", ";
      os << "Avg Leaf Entropy: " << m_trees[i]->get_average_leaf_entropy() << ", ";
      os << "Class Frequencies: " << m_trees[i]->get_class_frequencies() << '\n';
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
   * \brief Trains the forest by splitting a number of suitable nodes in each tree.
   *
   * The number of nodes that are split in each training step is limited to ensure that a step is not overly costly.
   *
   * \param splitBudget The maximum number of nodes per tree that may be split in this training step.
   * \return            The total number of nodes that have been split across trees.
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
