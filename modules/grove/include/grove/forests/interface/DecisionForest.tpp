/**
 * grove: DecisionForest.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "DecisionForest.h"

#include <fstream>

#include <boost/lexical_cast.hpp>

#ifdef WITH_SCOREFORESTS
#include <Learner.hpp>
#endif

#include <itmx/base/MemoryBlockFactory.h>

#include <tvgutil/numbers/RandomNumberGenerator.h>

// Whether or not to replace the pre-computed feature indices and thresholds with random ones.
#define RANDOM_FEATURES 0

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename DescriptorType, int TreeCount>
DecisionForest<DescriptorType,TreeCount>::DecisionForest()
: m_nbTotalLeaves(0)
{}

template <typename DescriptorType, int TreeCount>
DecisionForest<DescriptorType,TreeCount>::DecisionForest(const std::string& filename)
{
  load_structure_from_file(filename);
}

#ifdef WITH_SCOREFORESTS
template <typename DescriptorType, int TreeCount>
DecisionForest<DescriptorType, TreeCount>::DecisionForest(const EnsembleLearner& pretrainedForest)
{
  // Convert list of nodes into an appropriate image.
  const uint32_t nbTrees = pretrainedForest.GetNbTrees();
  const uint32_t maxNbNodes = pretrainedForest.GetMaxNbNodesInAnyLearner();

  if(nbTrees != get_nb_trees())
  {
    throw std::runtime_error("Number of trees in the loaded forest different from the instantiation of GPUForest.");
  }

  // Allocate the texture to store the nodes.
  const itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();
  m_nodeImage = mbf.make_image<NodeEntry>(Vector2i(nbTrees, maxNbNodes));
  m_nodeImage->Clear();

  // Fill the nodes.
  NodeEntry *forestData = m_nodeImage->GetData(MEMORYDEVICE_CPU);
  uint32_t totalNbLeaves = 0;

  for(uint32_t treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    const Learner *tree = pretrainedForest.GetTree(treeIdx);
    const uint32_t nbNodes = tree->GetNbNodes();

    // Bug in ScoreForests: tree->GetNbLeaves() always returns 1 for trees that have been loaded from a file because
    // the base learner class does not store the leaves and the DTBP class does not perform the loading (is done at the
    // base class level).
    // const int nbLeaves = tree->GetNbLeaves();

    // We have to count the number of leaves in each tree
    uint32_t nbLeavesBefore = totalNbLeaves;

    // Recursive call: we set the first free entry to 1, since we reserve 0 for the root of the tree.
    convert_node(tree, 0, treeIdx, nbTrees, 0, 1, forestData, totalNbLeaves);

    uint32_t nbLeaves = totalNbLeaves - nbLeavesBefore;

    std::cout << "Converted tree " << treeIdx << ", had " << nbNodes << " nodes and " << nbLeaves << " leaves." << std::endl;

    m_nbNodesPerTree.push_back(nbNodes);
    m_nbLeavesPerTree.push_back(nbLeaves);
  }

  // NOPs if we use the CPU only implementation
  m_nodeImage->UpdateDeviceFromHost();
}
#endif

//#################### DESTRUCTOR ####################

template <typename DescriptorType, int TreeCount>
DecisionForest<DescriptorType,TreeCount>::~DecisionForest()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType, TreeCount>::get_nb_leaves() const
{
  uint32_t nbLeaves = 0;
  for(uint32_t i = 0; i < get_nb_trees(); ++i)
  {
    nbLeaves += get_nb_leaves_in_tree(i);
  }
  return nbLeaves;
}

template <typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType,TreeCount>::get_nb_leaves_in_tree(uint32_t treeIdx) const
{
  if(treeIdx < get_nb_trees()) return m_nbLeavesPerTree[treeIdx];
  else throw std::invalid_argument("Invalid tree index");
}

template <typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType,TreeCount>::get_nb_nodes_in_tree(uint32_t treeIdx) const
{
  if(treeIdx < get_nb_trees()) return m_nbNodesPerTree[treeIdx];
  else throw std::invalid_argument("Invalid tree index");
}

template <typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType,TreeCount>::get_nb_trees() const
{
  return TREE_COUNT;
}

template <typename DescriptorType, int TreeCount>
void DecisionForest<DescriptorType,TreeCount>::load_structure_from_file(const std::string& filename)
{
  // Clear the current forest.
  m_nodeImage.reset();
  m_nbNodesPerTree.clear();
  m_nbLeavesPerTree.clear();
  m_nbTotalLeaves = 0;

  std::ifstream in(filename.c_str());
  if(!in) throw std::runtime_error("Couldn't load a forest from: " + filename);

  // Check that the number of trees is the same as the template instantiation.
  uint32_t nbTrees;
  in >> nbTrees;
  if(!in || nbTrees != get_nb_trees())
  {
    throw std::runtime_error(
      "Number of trees of the loaded forest is incorrect. Should be " +
      boost::lexical_cast<std::string>(get_nb_trees()) + " - Read: " +
      boost::lexical_cast<std::string>(nbTrees)
    );
  }

  // Used to allocate the indexing texture (height = the maximum number of nodes, width = nbTrees).
  uint32_t maxNbNodes = 0;

  // For each tree, first read the number of nodes, then the number of leaves.
  for(uint32_t i = 0; i < nbTrees; ++i)
  {
    uint32_t nbNodes, nbLeaves;
    in >> nbNodes >> nbLeaves;

    if(!in) throw std::runtime_error("Error reading the dimensions of tree: " + boost::lexical_cast<std::string>(i));

    m_nbNodesPerTree.push_back(nbNodes);
    m_nbLeavesPerTree.push_back(nbLeaves);

    maxNbNodes = std::max(nbNodes, maxNbNodes);
    m_nbTotalLeaves += nbLeaves;
  }

  std::cout << "Loading a forest with " << nbTrees << " trees.\n";
  for(uint32_t i = 0; i < nbTrees; ++i)
  {
    std::cout << "\tTree " << i << ": " << m_nbNodesPerTree[i] << " nodes and " << m_nbLeavesPerTree[i] << " leaves.\n";
  }

  // Allocate and clear the node image.
  const itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();
  m_nodeImage = mbf.make_image<NodeEntry>(Vector2i(nbTrees, maxNbNodes));
  m_nodeImage->Clear();

#if RANDOM_FEATURES
  tvgutil::RandomNumberGenerator rng(42);
#endif

  // Read all the nodes from the file.
  NodeEntry *forestNodes = m_nodeImage->GetData(MEMORYDEVICE_CPU);
  for(uint32_t treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    for(uint32_t nodeIdx = 0; nodeIdx < m_nbNodesPerTree[treeIdx]; ++nodeIdx)
    {
      NodeEntry &node = forestNodes[nodeIdx * nbTrees + treeIdx];
      in >> node.leftChildIdx >> node.leafIdx >> node.featureIdx >> node.featureThreshold;

      if(!in)
      {
        throw std::runtime_error(
          "Error reading node " + boost::lexical_cast<std::string>(nodeIdx) + " of tree " +
          boost::lexical_cast<std::string>(treeIdx)
        );
      }

#if RANDOM_FEATURES
      // The magic numbers mimic the distribution found in the pre-trained office forest.
      bool depthFeature = rng.generate_real_from_uniform(0.f, 1.f) < 0.3886f;

      if (depthFeature)
      {
        node.featureIdx = rng.generate_int_from_uniform(0, 127);

        float depthMu = 20.09f;
        float depthSigma = 947.24f;
        node.featureThreshold = rng.generate_from_gaussian(depthMu, depthSigma);
      }
      else
      {
        node.featureIdx = rng.generate_int_from_uniform(128, 255);

        float rgbMu = -2.85f;
        float rgbSigma = 72.98f;
        node.featureThreshold = rng.generate_from_gaussian(rgbMu, rgbSigma);
      }

//      int minRGBFeature = -100;
//      int maxRGBFeature = 100;
//      int minDepthFeature = -600;
//      int maxDepthFeature = 600;
//      node.featureIdx = rng.generate_int_from_uniform(0, RGBDPatchFeature::FEATURE_SIZE - 1);
//      if(node.featureIdx < RGBDPatchFeature::RGB_OFFSET)
//      {
//        node.featureThreshold = rng.generate_int_from_uniform(minDepthFeature, maxDepthFeature);
//      }
//      else
//      {
//        node.featureThreshold = rng.generate_int_from_uniform(minRGBFeature, maxRGBFeature);
//      }
#endif
    }
  }

  // Ensure that the node image is available on the GPU (if we're using it).
  m_nodeImage->UpdateDeviceFromHost();
}

template <typename DescriptorType, int TreeCount>
void DecisionForest<DescriptorType,TreeCount>::save_structure_to_file(const std::string& filename) const
{
  std::ofstream out(filename.c_str());

  // Write the number of trees.
  const uint32_t nbTrees = get_nb_trees();
  out << nbTrees << '\n';

  // For each tree, first write the number of nodes, then the number of leaves.
  for(uint32_t i = 0; i < nbTrees; ++i)
  {
    out << m_nbNodesPerTree[i] << ' ' << m_nbLeavesPerTree[i] << '\n';
  }

  // Then, for each tree, dump its nodes.
  const NodeEntry *forestNodes = m_nodeImage->GetData(MEMORYDEVICE_CPU);
  for(uint32_t treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    for(uint32_t nodeIdx = 0; nodeIdx < m_nbNodesPerTree[treeIdx]; ++nodeIdx)
    {
      const NodeEntry& node = forestNodes[nodeIdx * nbTrees + treeIdx];
      out << node.leftChildIdx << ' ' << node.leafIdx << ' ' << node.featureIdx << ' ' << std::setprecision(7) << node.featureThreshold << '\n';
    }
  }

  if(!out) throw std::runtime_error("Error saving the forest to a file: " + filename);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

#ifdef WITH_SCOREFORESTS
template <typename DescriptorType, int TreeCount>
int DecisionForest<DescriptorType, TreeCount>::convert_node(const Learner *tree, uint32_t nodeIdx, uint32_t treeIdx, uint32_t nbTrees, uint32_t outputIdx,
                                                            uint32_t outputFirstFreeIdx, NodeEntry *outputNodes, uint32_t& outputNbLeaves)
{
  const Node *node = tree->GetNode(nodeIdx);
  NodeEntry& outputNode = outputNodes[outputIdx * nbTrees + treeIdx];

  // The assumption is that outputIdx is already reserved for the current node.
  if(node->IsALeaf())
  {
    outputNode.leftChildIdx = -1; // Is a leaf
    outputNode.featureIdx = 0;
    outputNode.featureThreshold = 0.f;
    // outputFirstFreeIdx does not change

    // Post-increment to get the current leaf index.
    outputNode.leafIdx = outputNbLeaves++;
  }
  else
  {
    outputNode.leafIdx = -1; // Not a leaf

    // Reserve 2 entries for the child nodes.
    outputNode.leftChildIdx = outputFirstFreeIdx++;
    const uint32_t rightChildIdx =
        outputFirstFreeIdx++; // No need to store it in the texture since it's always leftChildIdx + 1

    // Use the ScoreForests cast to get the split parameters.
    const InnerNode *innerNode = ToInnerNode(node);
    std::vector<float> params = innerNode->GetFeature()->GetParameters();

    outputNode.featureIdx = params[1];
    outputNode.featureThreshold = params[2];

    // Recursively convert the left child and its descendants.
    outputFirstFreeIdx = convert_node(
      tree,
      node->GetLeftChildIndex(),
      treeIdx,
      nbTrees,
      outputNode.leftChildIdx,
      outputFirstFreeIdx,
      outputNodes,
      outputNbLeaves
    );

    // Same for right child and descendants.
    outputFirstFreeIdx = convert_node(
      tree,
      node->GetRightChildIndex(),
      treeIdx,
      nbTrees,
      rightChildIdx,
      outputFirstFreeIdx,
      outputNodes,
      outputNbLeaves
    );
  }

  return outputFirstFreeIdx;
}
#endif

}
