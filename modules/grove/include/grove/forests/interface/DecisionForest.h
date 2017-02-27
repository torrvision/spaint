/**
 * grove: DecisionForest.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFOREST
#define H_GROVE_DECISIONFOREST

#include <vector>
#include <boost/shared_ptr.hpp>

#include <ORUtils/Image.h>

#include <spaint/util/ITMImagePtrTypes.h>

namespace grove {

template <typename DescriptorType, int TreeCount>
class DecisionForest
{
public:
  enum { TREE_COUNT = TreeCount };

  typedef ORUtils::Image<DescriptorType> DescriptorImage;
  typedef boost::shared_ptr<DescriptorImage> DescriptorImage_Ptr;
  typedef boost::shared_ptr<const DescriptorImage> DescriptorImage_CPtr;

  typedef ORUtils::VectorX<int, TREE_COUNT> LeafIndices;
  typedef ORUtils::Image<LeafIndices> LeafIndicesImage;
  typedef boost::shared_ptr<LeafIndicesImage> LeafIndicesImage_Ptr;
  typedef boost::shared_ptr<const LeafIndicesImage> LeafIndicesImage_CPtr;

  struct NodeEntry
  {
    uint32_t leftChildIdx;  // No need to store the right child, it's left + 1
    int leafIdx;            // Index of the associated leaf (-1 if the node is not a leaf)
    uint32_t featureIdx;    // Index of the feature to evaluate;
    float featureThreshold; // Feature threshold
  };

  typedef ORUtils::Image<NodeEntry> NodeImage;
  typedef boost::shared_ptr<ORUtils::Image<NodeEntry> > NodeImage_Ptr;
  typedef boost::shared_ptr<const ORUtils::Image<NodeEntry> > NodeImage_CPtr;

public:
  DecisionForest(const std::string& fileName);
  virtual ~DecisionForest();

  virtual void find_leaves(const DescriptorImage_CPtr& descriptors, LeafIndicesImage_Ptr& leafIndices) const = 0;

  uint32_t get_nb_trees() const;
  uint32_t get_nb_leaves() const;
  uint32_t get_nb_nodes_in_tree(uint32_t treeIdx) const;
  uint32_t get_nb_leaves_in_tree(uint32_t treeIdx) const;

  void load_structure_from_file(const std::string &fileName);
  void save_structure_to_file(const std::string &fileName) const;

protected:
  DecisionForest();

protected:
  std::vector<uint32_t> m_nbNodesPerTree;
  std::vector<uint32_t> m_nbLeavesPerTree;

  uint32_t m_nbTotalLeaves;

  NodeImage_Ptr m_nodeImage;

};

}

#endif
