/**
 * spaint: GPUForest.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/GPUForest.h"

#include <boost/make_shared.hpp>

#include "util/MemoryBlockFactory.h"

namespace spaint
{
GPUForest::GPUForest(const EnsembleLearner &pretrained_forest)
{
  // Convert list of nodes into an appropriate image
  const int nTrees = pretrained_forest.GetNbTrees();
  const int maxNbNodes = pretrained_forest.GetMaxNbNodesInAnyLearner();

  if (nTrees != NTREES)
  {
    throw std::runtime_error(
        "Number of trees in the loaded forest different from the instantiation of GPUForest.");
  }

  // Create texture storing the nodes
  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_forestImage = mbf.make_image<GPUForestNode>(Vector2i(nTrees, maxNbNodes));
  m_forestImage->Clear();

  std::cout << "Forest texture has size: " << m_forestImage->noDims
      << std::endl;

  // Fill the nodes
  GPUForestNode *forestData = m_forestImage->GetData(MEMORYDEVICE_CPU);

  for (int treeIdx = 0; treeIdx < nTrees; ++treeIdx)
  {
    const Learner* tree = pretrained_forest.GetTree(treeIdx);
    const int nbNodes = tree->GetNbNodes();

    // We set the first free entry to 1 since we reserve 0 for the root
    int first_free_idx = convert_node(tree, 0, treeIdx, nTrees, 0, 1,
        forestData);
    std::cout << "Converted tree " << treeIdx << ", had " << nbNodes
        << " nodes." << std::endl;
    std::cout << "Total number of leaves: " << m_leafPredictions.size()
        << std::endl;
  }
}

GPUForest::~GPUForest()
{
}

int GPUForest::convert_node(const Learner *tree, int node_idx, int tree_idx,
    int n_trees, int output_idx, int first_free_idx, GPUForestNode *gpu_nodes)
{
  const Node* node = tree->GetNode(node_idx);
  GPUForestNode &gpuNode = gpu_nodes[output_idx * n_trees + tree_idx];

  // The assumption is that output_idx is already reserved for the current node
  if (node->IsALeaf())
  {
//    gpuNode.leafIdx = node_idx; // Node index in the original tree, could be used to get the modes from there
    gpuNode.leftChildIdx = -1; // Is a leaf
    gpuNode.featureIdx = 0;
    gpuNode.featureThreshold = 0.f;
    // first_free_idx does not change

    gpuNode.leafIdx = m_leafPredictions.size();

    // Copy the prediction
    // TODO: possibly drop some modes
    const LeafBPDGaussianMean* leafPtr = ToLeafBPDGaussianMean(node);
    if (leafPtr->GetPrediction())
    {
      const PredictionGaussianMean *pred = ToPredictionGaussianMean(
          leafPtr->GetPrediction());
      m_leafPredictions.push_back(*pred);
    }
    else
    {
      // empty prediction
      m_leafPredictions.push_back(PredictionGaussianMean());
    }

  }
  else
  {
    gpuNode.leafIdx = -1; // Not a leaf

    // Reserve 2 slots for the child nodes.
    gpuNode.leftChildIdx = first_free_idx++;
    int rightChildIdx = first_free_idx++; // No need to store it in the texture since it's always leftChildIdx + 1

    const InnerNode *inner_node = ToInnerNode(node);
    std::vector<float> params = inner_node->GetFeature()->GetParameters();

    gpuNode.featureIdx = params[1];
    gpuNode.featureThreshold = params[2];

    first_free_idx = convert_node(tree, node->GetLeftChildIndex(), tree_idx,
        n_trees, gpuNode.leftChildIdx, first_free_idx, gpu_nodes);
    first_free_idx = convert_node(tree, node->GetRightChildIndex(), tree_idx,
        n_trees, rightChildIdx, first_free_idx, gpu_nodes);
  }

  return first_free_idx;
}

boost::shared_ptr<EnsemblePredictionGaussianMean> GPUForest::get_prediction_for_leaves(
    const LeafIndices &leaves)
{
  boost::shared_ptr<EnsemblePredictionGaussianMean> res = boost::make_shared<
      EnsemblePredictionGaussianMean>();

  for (int treeIdx = 0; treeIdx < NTREES; ++treeIdx)
  {
    PredictionGaussianMean &currentPred = m_leafPredictions[leaves[treeIdx]];
    const size_t nbModes = currentPred._modes.size();

    res->_modes.reserve(res->_modes.size() + nbModes); // Make some space

//    res->_modes.push_back(std::vector<PredictedGaussianMean*>(nbModes));
    for (size_t modeIdx = 0; modeIdx < nbModes; ++modeIdx)
    {
      // Is a vector because [0] is the position and [1] is the colour of the mode
      std::vector<PredictedGaussianMean> &currentMode =
          currentPred._modes[modeIdx];
      const size_t nbComponents = currentMode.size();

      res->_modes.push_back(std::vector<PredictedGaussianMean*>(nbComponents));
      for (size_t compIdx = 0; compIdx < nbComponents; ++compIdx)
      {
        res->_modes.back()[compIdx] = &currentMode[compIdx];
      }
    }
  }

  return res;
}

}
