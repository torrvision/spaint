/**
 * grove: DecisionForest.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "DecisionForest.h"

#include <fstream>
#include <boost/lexical_cast.hpp>

#include <spaint/util/MemoryBlockFactory.h>
using spaint::MemoryBlockFactory;

#include <tvgutil/numbers/RandomNumberGenerator.h>
using tvgutil::RandomNumberGenerator;

// Whether to replace the pre-computed feature indices and thresholds with random ones.
#define RANDOM_FEATURES 0

namespace grove {

template<typename DescriptorType, int TreeCount>
DecisionForest<DescriptorType, TreeCount>::DecisionForest()
  : m_nbTotalLeaves(0)
{}

template<typename DescriptorType, int TreeCount>
DecisionForest<DescriptorType, TreeCount>::DecisionForest(const std::string& fileName)
  : DecisionForest()
{
  load_structure_from_file(fileName);
}

template<typename DescriptorType, int TreeCount>
DecisionForest<DescriptorType, TreeCount>::~DecisionForest()
{}

template<typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType, TreeCount>::get_nb_leaves() const
{
  uint32_t nbLeaves = 0;

  for(uint32_t i = 0; i < get_nb_trees(); ++i)
    nbLeaves += get_nb_leaves_in_tree(i);

  return nbLeaves;
}

template<typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType, TreeCount>::get_nb_trees() const
{
  return TREE_COUNT;
}

template<typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType, TreeCount>::get_nb_nodes_in_tree(uint32_t treeIdx) const
{
  if (treeIdx >= get_nb_trees())
    throw std::invalid_argument("invalid treeIdx");

  return m_nbNodesPerTree[treeIdx];
}

template<typename DescriptorType, int TreeCount>
uint32_t DecisionForest<DescriptorType, TreeCount>::get_nb_leaves_in_tree(uint32_t treeIdx) const
{
  if (treeIdx >= get_nb_trees())
    throw std::invalid_argument("invalid treeIdx");

  return m_nbLeavesPerTree[treeIdx];
}

template<typename DescriptorType, int TreeCount>
void DecisionForest<DescriptorType, TreeCount>::load_structure_from_file(const std::string &fileName)
{
  // clean current forest
  m_nodeImage.reset();
  m_nbNodesPerTree.clear();
  m_nbLeavesPerTree.clear();
  m_nbTotalLeaves = 0;

  std::ifstream in(fileName);

  if (!in)
    throw std::runtime_error("Couldn't load a forest from: " + fileName);

  // Check number of trees
  uint32_t nbTrees;
  in >> nbTrees;
  if (!in || nbTrees != get_nb_trees())
    throw std::runtime_error(
        "Number of trees of the loaded forest is incorrect. Should be "
            + boost::lexical_cast<std::string>(get_nb_trees()) + " - Read: "
            + boost::lexical_cast<std::string>(nbTrees));

  // Read number of nodes and leaves
  uint32_t maxNbNodes = 0;    // Used to allocate the texture (needs to have the height equal to the maximum number of nodes)

  // For each tree read first the number of nodes, then the number of leaves.
  for (uint32_t i = 0; i < nbTrees; ++i)
  {
    uint32_t nbNodes, nbLeaves;
    in >> nbNodes >> nbLeaves;

    if (!in)
      throw std::runtime_error("Error reading the dimensions of tree: " + boost::lexical_cast<std::string>(i));

    m_nbNodesPerTree.push_back(nbNodes);
    m_nbLeavesPerTree.push_back(nbLeaves);

    maxNbNodes = std::max(nbNodes, maxNbNodes);
    m_nbTotalLeaves += nbLeaves;
  }

  std::cout << "Loading a forest with " << nbTrees << " trees.\n";
  for (uint32_t i = 0; i < nbTrees; ++i)
  {
    std::cout << "\tTree " << i << ": " << m_nbNodesPerTree[i] << " nodes and "
        << m_nbLeavesPerTree[i] << " leaves.\n";
  }

  // Allocate data
  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_nodeImage = mbf.make_image<NodeEntry>(Vector2i(nbTrees, maxNbNodes));
  m_nodeImage->Clear();

#if RANDOM_FEATURES
  tvgutil::RandomNumberGenerator rng(42);
#endif

  // Read all nodes.
  NodeEntry *forestData = m_nodeImage->GetData(MEMORYDEVICE_CPU);
  for (uint32_t treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    for (uint32_t nodeIdx = 0; nodeIdx < m_nbNodesPerTree[treeIdx]; ++nodeIdx)
    {
      NodeEntry& node = forestData[nodeIdx * nbTrees + treeIdx];
      in >> node.leftChildIdx >> node.leafIdx >> node.featureIdx >> node.featureThreshold;

      if (!in)
        throw std::runtime_error(
            "Error reading node " + boost::lexical_cast<std::string>(nodeIdx)
                + " of tree " + boost::lexical_cast<std::string>(treeIdx));

#if RANDOM_FEATURES
      // Magic numbers to mimic the distribution found in the pre-trained office forest.

      bool depthFeature = rng.generate_real_from_uniform(0.f, 1.f) < 0.3886f;

      if(depthFeature)
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

  // Update device forest on the device (if necessary).
  m_nodeImage->UpdateDeviceFromHost();
}

template<typename DescriptorType, int TreeCount>
void DecisionForest<DescriptorType, TreeCount>::save_structure_to_file(const std::string &fileName) const
{
  const uint32_t nbTrees = get_nb_trees();
  std::ofstream out(fileName, std::ios::trunc);

  // Write the number of trees
  out << nbTrees << '\n';

  // For each tree write first the number of nodes, then the number of leaves.
  for (uint32_t i = 0; i < nbTrees; ++i)
  {
    out << m_nbNodesPerTree[i] << ' ' << m_nbLeavesPerTree[i] << '\n';
  }

  // Then, for each tree, dump its nodes.
  const NodeEntry *forestData = m_nodeImage->GetData(MEMORYDEVICE_CPU);
  for (uint32_t treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    for (uint32_t nodeIdx = 0; nodeIdx < m_nbNodesPerTree[treeIdx]; ++nodeIdx)
    {
      const NodeEntry& node = forestData[nodeIdx * nbTrees + treeIdx];
      out << node.leftChildIdx << ' ' << node.leafIdx << ' ' << node.featureIdx
          << ' ' << std::setprecision(7) << node.featureThreshold << '\n';
    }
  }

  if(!out) throw std::runtime_error("Error saving the forest to a file: " + fileName);
}

}
