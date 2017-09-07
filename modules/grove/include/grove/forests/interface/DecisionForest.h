/**
 * grove: DecisionForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFOREST
#define H_GROVE_DECISIONFOREST

#include <vector>

#include <boost/shared_ptr.hpp>

#include <ORUtils/Image.h>

//#################### FORWARD DECLARATIONS ####################

#ifdef WITH_SCOREFORESTS
class EnsembleLearner;
class Learner;
class PredictionGaussianMean;
#endif

namespace grove {

/**
 * \brief An instance of a class deriving from this one represents a binary decision forest composed of a fixed number of trees.
 *
 * \note  Training is not performed by this class. We use the node indexing technique described in
 *        "Implementing Decision Trees and Forests on a GPU" (Toby Sharp, 2008).
 *
 * \tparam DescriptorType The type of descriptor used to find the leaves. Must have a floating-point member array named "data".
 * \tparam TreeCount      The number of trees in the forest. Fixed at compilation time to allow the definition of a data type
 *                        representing the leaf indices.
 */
template <typename DescriptorType, int TreeCount>
class DecisionForest
{
  //#################### ENUMERATIONS ####################
public:
  // Expose the tree count to client code.
  enum { TREE_COUNT = TreeCount };

  //#################### NESTED TYPES ####################
public:
  /**
   * \brief An instance of this struct represents a single node in a forest tree.
   *
   * \note Each branch node stores the parameters of a decision function that tests an individual feature against a threshold.
   */
  struct NodeEntry
  {
    /**
     * The index of the feature in a feature descriptor that should be compared to the threshold.
     * If the node is a leaf, this is set to 0.
     */
    uint32_t featureIdx;

    /**
     * The threshold against which to compare the feature. When routing a descriptor down the tree,
     * we descend to the left child if descriptor.data[featureIdx] < featureThreshold, and descend
     * to the right child otherwise. If the node is a leaf, this is set to 0.
     */
    float featureThreshold;

    /** The index of the leaf associated with the node, if any, or -1 if the node is a branch. */
    int leafIdx;

    /**
     * The index of the node's left child, if any, or -1 if the node is a leaf. Note that we don't
     * need to store the index of the right child, because it's always either 1 + leftChildIdx or -1.
     */
    int leftChildIdx;
  };

  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::Image<DescriptorType> DescriptorImage;
  typedef boost::shared_ptr<DescriptorImage> DescriptorImage_Ptr;
  typedef boost::shared_ptr<const DescriptorImage> DescriptorImage_CPtr;
  typedef ORUtils::VectorX<int,TREE_COUNT> LeafIndices;
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;
private:
  typedef ORUtils::Image<NodeEntry> NodeImage;
  typedef boost::shared_ptr<ORUtils::Image<NodeEntry> > NodeImage_Ptr;

  //#################### PROTECTED MEMBER VARIABLES ####################
protected:
  /** The number of leaves in each tree. */
  std::vector<uint32_t> m_nbLeavesPerTree;

  /** The number of nodes in each tree. */
  std::vector<uint32_t> m_nbNodesPerTree;

  /** The total number of leaves in the forest. */
  uint32_t m_nbTotalLeaves;

  /** An image storing the indexing structure of the forest. See the paper by Toby Sharp for details. */
  NodeImage_Ptr m_nodeImage;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an empty decision forest.
   */
  DecisionForest();

  /**
   * \brief Loads the branching structure of a pre-trained decision forest from a file on disk.
   *
   * \param filename The path to the file containing the forest.
   *
   * \throws std::runtime_error If the forest cannot be loaded.
   */
  explicit DecisionForest(const std::string& filename);

#ifdef WITH_SCOREFORESTS
  /**
   * \brief Constructs a decision forest by converting an EnsembleLearner that was pre-trained using ScoreForests.
   *
   * \param pretrainedForest The pre-trained forest to convert.
   *
   * \throws std::runtime_error If the pre-trained forest cannot be converted.
   */
  explicit DecisionForest(const EnsembleLearner& pretrainedForest);
#endif

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the decision forest.
   */
  virtual ~DecisionForest();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Given an image filled with descriptors, evaluates the forest and returns the leaf indices associated with each descriptor (one per tree).
   *
   * \param descriptors An image in which each pixel contains a descriptor. All descriptors are assumed valid and are fed to every tree in the forest.
   * \param leafIndices An image (of the same size as descriptors) in which to store the leaf indices computed for each descriptor.
   */
  virtual void find_leaves(const DescriptorImage_CPtr& descriptors, LeafIndicesImage_Ptr& leafIndices) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the total number of leaves in the forest.
   *
   * \return The total number of leaves in the forest.
   */
  uint32_t get_nb_leaves() const;

  /**
   * \brief Gets the number of leaves in the specified tree.
   *
   * \param treeIdx The index of the tree whose leaves we wish to count.
   * \return        The number of leaves in the specified tree.
   *
   * \throws std::invalid_argument If treeIdx is >= than the actual number of trees.
   */
  uint32_t get_nb_leaves_in_tree(uint32_t treeIdx) const;

  /**
   * \brief Gets the number of nodes in the specified tree.
   *
   * \param treeIdx The index of the tree whose nodes we wish to count.
   * \return        The number of nodes in the specified tree.
   *
   * \throws std::invalid_argument If treeIdx is >= than the actual number of trees.
   */
  uint32_t get_nb_nodes_in_tree(uint32_t treeIdx) const;

  /**
   * \brief Gets the number of trees in the forest.
   *
   * \return The number of trees in the forest.
   */
  uint32_t get_nb_trees() const;

  /**
   * \brief Loads the branching structure of a pre-trained decision forest from a file on disk.
   *
   * \param filename  The path to the file containing the forest.
   *
   * \throws std::runtime_error If the forest cannot be loaded.
   *
   * \note File format (text mode):
   *
   * nbTrees
   * tree1_nbNodes tree1_nbLeaves
   * ...
   * treeN_nbNodes treeN_nbLeaves
   * tree1_node1_leftChildIdx tree1_node1_leafIdx tree1_node1_featureIdx tree1_node1_featureThreshold
   * ...
   * tree1_nodeN_leftChildIdx tree1_nodeN_leafIdx tree1_nodeN_featureIdx tree1_nodeN_featureThreshold
   * ...
   * treeN_node1_leftChildIdx treeN_node1_leafIdx treeN_node1_featureIdx treeN_node1_featureThreshold
   * ...
   * treeN_nodeN_leftChildIdx treeN_nodeN_leafIdx treeN_nodeN_featureIdx treeN_nodeN_featureThreshold
   */
  void load_structure_from_file(const std::string& filename);

  /**
   * \brief Saves the branching structure of the decision forest to a file on disk.
   *
   * \param filename  The path to the file to which to save the forest.
   *
   * \throws std::runtime_error If the forest cannot be saved.
   */
  void save_structure_to_file(const std::string& filename) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
#ifdef WITH_SCOREFORESTS
  /**
   * \brief Converts a single node from a tree that was pre-trained with ScoreForests.
   *        This function is recursive: if called with the root of a tree, it converts the entire tree.
   *
   * \param learner            The ScoreForests tree.
   * \param nodeIdx            The index of the ScoreForests node to convert.
   * \param treeIdx            The index of the tree in the forest in which we want to store the converted node.
   * \param nbTrees            The number of trees in the forest, used to compute offsets in the outputNodes array.
   * \param outputIdx          The y-index of the node in outputNodes (x coordinate: treeIdx) where we want to store the result of the conversion.
   * \param outputFirstFreeIdx The first free y-index in outputNodes (x coordinate: treeIdx), used to allocate the node children.
   * \param outputNodes        A pointer to the m_nodeImage data. Nodes of each tree are stored in columns of the image.
   * \param outputNbLeaves     Input-Output: the content of the variable is increased by the total number of leaves
   *                           descending from the current node. Used to allocate unique leaf identifiers.
   *
   * \return The y-index of the first free entry after the conversion of the node and all its descendants.
   */
  int convert_node(const Learner *learner, uint32_t nodeIdx, uint32_t treeIdx, uint32_t nbTrees, uint32_t outputIdx,
                   uint32_t outputFirstFreeIdx, NodeEntry *outputNodes, uint32_t& outputNbLeaves);
#endif
};

}

#endif
