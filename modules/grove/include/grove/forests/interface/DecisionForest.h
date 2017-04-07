/**
 * grove: DecisionForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFOREST
#define H_GROVE_DECISIONFOREST

#include <vector>

#include <boost/shared_ptr.hpp>

#include <ORUtils/Image.h>

#include <itmx/ITMImagePtrTypes.h>

#ifdef WITH_SCOREFORESTS
// Forward declare stuff, to avoid cluttering the global namespace with every ScoreForest class.
class EnsembleLearner;
class Learner;
class PredictionGaussianMean;
#endif

namespace grove {

/**
 * \brief An instance of a class deriving from this one represents a binary decision forest composed of a fixed number
 *        of trees.
 *
 * \note  Training is not performed by this class. We use the node indexing technique described in:
 *        "Toby Sharp, Implementing decision trees and forests on a GPU. (2008)"
 *
 * \param DescriptorType  The type of descriptor used to find the leaves. Must have a floating-point member array named
 *                        "data".
 * \param TreeCount       The number of trees in the forest. Fixed at compilation time to allow the definition of a data
 *                        type representing the leaf indices.
 */
template <typename DescriptorType, int TreeCount>
class DecisionForest
{
  //#################### ENUMS ####################
public:
  enum { TREE_COUNT = TreeCount };

  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::Image<DescriptorType> DescriptorImage;
  typedef boost::shared_ptr<DescriptorImage> DescriptorImage_Ptr;
  typedef boost::shared_ptr<const DescriptorImage> DescriptorImage_CPtr;

  typedef ORUtils::VectorX<int, TREE_COUNT> LeafIndices;
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

  //#################### NESTED STRUCTS ####################
public:
  /**
   * \brief An instance of this struct represents a single node in a forest tree.
   */
  struct NodeEntry
  {
    /** Index of the feature to evaluate. */
    uint32_t featureIdx;

    /** The threshold used to select the child when evaluating the node:
     *  if descriptor.data[featureIdx] < featureThreshold then childIdx = leaftChildIdx else childIdx = leftChildIdx +
     * 1.
     */
    float featureThreshold; // Feature threshold

    /** The index of the leaf associated to the node. If the node is not a leaf, set to -1. */
    int leafIdx;

    /**
     * \brief Index of the node's left child. We don't need to store the right child index because it's always
     *        leftChildIdx + 1. Set to -1 if the node is a leaf.
     */
    int leftChildIdx;
  };

  //#################### TYPEDEFS ####################
public:
  typedef ORUtils::Image<NodeEntry> NodeImage;
  typedef boost::shared_ptr<ORUtils::Image<NodeEntry> > NodeImage_Ptr;
  typedef boost::shared_ptr<const ORUtils::Image<NodeEntry> > NodeImage_CPtr;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an empty DecisionForest.
   */
  DecisionForest();

  /**
   * \brief Constructs an instance of a pretrained DecisionForest.
   *
   * \param fileName The path to a file representing a decision forest.
   *
   * \throws std::runtime_error if the forest cannot be loaded.
   */
  explicit DecisionForest(const std::string &fileName);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destructs a DecisionForest.
   */
  virtual ~DecisionForest();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Given an image filled with descriptors, evaluates the forest and returns the leaf indices associated to each
   *        descriptor.
   *
   * \param descriptors An image wherein each element represents a descriptor. All elements are assumed valid and are
   *                    fed to every tree in the forest.
   * \param leafIndices An image (of the same size as descriptors) wherein each element holds the TreeCount indices of
   *                    the leaves determined by the corresponding descriptor.
   */
  virtual void find_leaves(const DescriptorImage_CPtr &descriptors, LeafIndicesImage_Ptr &leafIndices) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the total number of leaves in the forest.
   *
   * \return The number of leaves in the forest.
   */
  uint32_t get_nb_leaves() const;

  /**
   * \brief Gets the number of leaves in a tree.
   *
   * \param treeIdx The tree of interest.
   *
   * \return The number of leaves in the tree.
   *
   * \throws std::invalid_argument if the treeIdx is >= than the actual number of trees.
   */
  uint32_t get_nb_leaves_in_tree(uint32_t treeIdx) const;

  /**
   * \brief Gets the number of nodes in a tree.
   *
   * \param treeIdx The tree of interest.
   *
   * \return The number of nodes in the tree.
   *
   * \throws std::invalid_argument if the treeIdx is >= than the actual number of trees.
   */
  uint32_t get_nb_nodes_in_tree(uint32_t treeIdx) const;

  /**
   * \brief Gets the total number of trees in the forest.
   *
   * \return The number of trees in the forest.
   */
  uint32_t get_nb_trees() const;

  /**
   * \brief Loads the forest from a file.
   *
   * \param fileName The path to the file containing the forest definition.
   *
   * \throws std::runtime_error if the forest cannot be loaded.
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
   *
   */
  void load_structure_from_file(const std::string &fileName);

  /**
   * \brief Saves the forest into a file.
   *
   * \param fileName The path to the file that will containin the forest definition.
   *
   * \throws std::runtime_error if the forest cannot be saved.
   */
  void save_structure_to_file(const std::string &fileName) const;

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

//#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a DecisionForest converting an EnsembleLearner pretrained by the ScoreForests project.
   *
   * \param pretrainedForest The pretrained forest that will be converted into a DecisionForest.
   *
   * \throws std::runtime_error if the pretrained forest cannot be converted.
   */
  explicit DecisionForest(const EnsembleLearner &pretrainedForest);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Converts a single node from a ScoreForest pretrained tree.
   *        This function is recursive, if called with the root of a tree converts the entire tree.
   *
   * \param learner            The ScoreForest tree.
   * \param nodeIdx            The index of the ScoreForest node to convert.
   * \param treeIdx            The index of the tree in the forest where we want to store the converted node.
   * \param nbTrees            The number of trees in the forest, used to compute offsets in the outputNodes array.
   * \param outputIdx          The y-index of the node in outputNodes (x coordinate: treeIdx) where we want to store the
   *                           result of the conversion.
   * \param outputFirstFreeIdx The first free y-index in outputNodes (x coordinate: treeIdx), used to allocate the node
   *                           childs.
   * \param outputNodes        A pointer to the m_nodeImage data. Nodes of each tree are stored in columns of the image.
   * \param outputNbLeaves     Input-Output: the content of the variable is increased by the total number of leaves
   *                           descending from the current node. Used to allocate unique leaf identifiers.
   *
   * \return The y-index of the first free entry after the conversion of the node and all its descendents.
   */
  int convert_node(const Learner *learner,
                   uint32_t nodeIdx,
                   uint32_t treeIdx,
                   uint32_t nbTrees,
                   uint32_t outputIdx,
                   uint32_t outputFirstFreeIdx,
                   NodeEntry *outputNodes,
                   uint32_t &outputNbLeaves);
#endif
};

} // namespace grove

#endif
