/**
 * spaint: GPUForest.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/GPUForest.h"

#include <fstream>
#include <iomanip>
#include <stdexcept>

#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/timer/timer.hpp>

#include "util/MemoryBlockFactory.h"

#include "randomforest/cuda/GPUClusterer_CUDA.h"
#include "randomforest/cuda/GPUReservoir_CUDA.h"

//#define ENABLE_TIMERS
//#define RANDOM_FEATURES

#ifdef RANDOM_FEATURES
#include "tvgutil/numbers/RandomNumberGenerator.h"
#endif

namespace spaint
{
GPUForest::GPUForest()
{
  // Tentative values
  const float clustererSigma = 0.1f;
  const float clustererTau = 0.05f;
  const int minClusterSize = 20;
  m_gpuClusterer.reset(
      new GPUClusterer_CUDA(clustererSigma, clustererTau, minClusterSize));

  // Allocate the image that will store the leaf indices (dummy size, will be resized as needed)
  m_leafImage = MemoryBlockFactory::instance().make_image<LeafIndices>(
      Vector2i(0, 0));

  m_lastFeaturesAddedStartIdx = 0;
  m_maxReservoirsToUpdate = 256;
  m_reservoirUpdateStartIdx = 0;
}

GPUForest::GPUForest(const std::string &fileName) :
    GPUForest()
{
  load_structure_from_file(fileName);
}

GPUForest::~GPUForest()
{
}

void GPUForest::evaluate_forest(const RGBDPatchFeatureImage_CPtr &features,
    GPUForestPredictionsImage_Ptr &predictions)
{
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "evaluating forest on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    find_leaves(features, m_leafImage);
  }

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "generating ensemble predictions on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    get_predictions(m_leafImage, predictions);
  }
}

void GPUForest::reset_predictions()
{
  m_predictionsBlock->Clear(); // Setting nbModes to 0 for each prediction would be enough.
  m_leafReservoirs->clear();
}

void GPUForest::add_features_to_forest(
    const RGBDPatchFeatureImage_CPtr &features)
{
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "evaluating forest on the GPU: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    find_leaves(features, m_leafImage);
//    cudaDeviceSynchronize();
  }

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "add examples to reservoirs: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_leafReservoirs->add_examples(features, m_leafImage);
//    cudaDeviceSynchronize();
  }

  const int updateCount = std::min<int>(m_maxReservoirsToUpdate,
      m_predictionsBlock->dataSize - m_reservoirUpdateStartIdx);

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "GPU clustering: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_gpuClusterer->find_modes(m_leafReservoirs, m_predictionsBlock,
        m_reservoirUpdateStartIdx, updateCount);
//    cudaDeviceSynchronize();
  }

//  std::cout << "Updated " << updateCount << " reservoirs, starting from "
//      << m_reservoirUpdateStartIdx << std::endl;

  // m_maxReservoirsToUpdate reservoirs starting from this index have been updated
  m_lastFeaturesAddedStartIdx = m_reservoirUpdateStartIdx;

  m_reservoirUpdateStartIdx += m_maxReservoirsToUpdate;
  if (m_reservoirUpdateStartIdx >= m_predictionsBlock->dataSize)
    m_reservoirUpdateStartIdx = 0;
}

void GPUForest::update_forest()
{
  // We are back to the first reservoir that was updated when
  // the last batch of features were added to the forest.
  // No need to update further, we would get the same modes.
  // This check works only if the m_maxReservoirsToUpdate
  // remains constant throughout the whole program.
  if (m_reservoirUpdateStartIdx == m_lastFeaturesAddedStartIdx)
    return;

  const int updateCount = std::min<int>(m_maxReservoirsToUpdate,
      m_predictionsBlock->dataSize - m_reservoirUpdateStartIdx);

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "GPU clustering: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_gpuClusterer->find_modes(m_leafReservoirs, m_predictionsBlock,
        m_reservoirUpdateStartIdx, updateCount);
//    cudaDeviceSynchronize();
  }

//  std::cout << "Updated " << updateCount << " reservoirs, starting from "
//      << m_reservoirUpdateStartIdx << std::endl;

  m_reservoirUpdateStartIdx += m_maxReservoirsToUpdate;
  if (m_reservoirUpdateStartIdx >= m_predictionsBlock->dataSize)
    m_reservoirUpdateStartIdx = 0;
}

void GPUForest::load_structure_from_file(const std::string &fileName)
{
  // clean current forest (TODO: put in a function)
  m_forestImage.reset();
  m_predictionsBlock.reset();
  m_leafReservoirs.reset();

  m_nbNodesPerTree.clear();
  m_nbLeavesPerTree.clear();
  m_reservoirUpdateStartIdx = 0;

  std::ifstream in(fileName);

  if (!in)
    throw std::runtime_error("Couldn't load a forest from: " + fileName);

  // Check number of trees
  int nbTrees;
  in >> nbTrees;
  if (!in || nbTrees != GPUFOREST_NTREES)
    throw std::runtime_error(
        "Number of trees of the loaded forest is incorrect. Should be "
            + boost::lexical_cast<std::string>(GPUFOREST_NTREES) + " - Read: "
            + boost::lexical_cast<std::string>(nbTrees));

  // Read number of nodes and leaves

  int maxNbNodes = 0; // Used to allocate the texture
  int totalNbLeaves = 0; // Used to allocate predictions and reservoirs

  // For each tree write first the number of nodes then the number of leaves
  for (int i = 0; i < nbTrees; ++i)
  {
    int nbNodes, nbLeaves;
    in >> nbNodes >> nbLeaves;

    if (!in)
      throw std::runtime_error(
          "Error reading the dimensions of tree: "
              + boost::lexical_cast<std::string>(i));

    m_nbNodesPerTree.push_back(nbNodes);
    m_nbLeavesPerTree.push_back(nbLeaves);

    maxNbNodes = std::max(nbNodes, maxNbNodes);
    totalNbLeaves += nbLeaves;
  }

  std::cout << "Loading a forest with " << nbTrees << " trees.\n";
  for (int i = 0; i < nbTrees; ++i)
  {
    std::cout << "\tTree " << i << ": " << m_nbNodesPerTree[i] << " nodes and "
        << m_nbLeavesPerTree[i] << " leaves.\n";
  }

  // Allocate data
  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_forestImage = mbf.make_image<GPUForestNode>(Vector2i(nbTrees, maxNbNodes));
  m_forestImage->Clear();

  m_predictionsBlock = mbf.make_block<GPUForestPrediction>(totalNbLeaves);
  m_predictionsBlock->Clear();

  m_leafReservoirs.reset(new GPUReservoir_CUDA(RESERVOIR_SIZE, totalNbLeaves));

#ifdef RANDOM_FEATURES
  tvgutil::RandomNumberGenerator rng(42);
#endif

  // Read all nodes.
  GPUForestNode *forestData = m_forestImage->GetData(MEMORYDEVICE_CPU);
  for (int treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    for (int nodeIdx = 0; nodeIdx < m_nbNodesPerTree[treeIdx]; ++nodeIdx)
    {
      GPUForestNode& node = forestData[nodeIdx * nbTrees + treeIdx];
      in >> node.leftChildIdx >> node.leafIdx >> node.featureIdx
          >> node.featureThreshold;

      if (!in)
        throw std::runtime_error(
            "Error reading node " + boost::lexical_cast<std::string>(nodeIdx)
                + " of tree " + boost::lexical_cast<std::string>(treeIdx));

#ifdef RANDOM_FEATURES
      // Mimic the distribution in the pretrained forest

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

  // Update device forest
  m_forestImage->UpdateDeviceFromHost();
}

void GPUForest::save_structure_to_file(const std::string &fileName) const
{
  std::ofstream out(fileName, std::ios::trunc);

  // Write the number of trees
  out << GPUFOREST_NTREES << '\n';

  // For each tree write first the number of nodes then the number of leaves
  for (size_t i = 0; i < GPUFOREST_NTREES; ++i)
  {
    out << m_nbNodesPerTree[i] << ' ' << m_nbLeavesPerTree[i] << '\n';
  }

  // Then, for each tree, dump its nodes
  const GPUForestNode *forestData = m_forestImage->GetData(MEMORYDEVICE_CPU);
  for (int treeIdx = 0; treeIdx < GPUFOREST_NTREES; ++treeIdx)
  {
    for (int nodeIdx = 0; nodeIdx < m_nbNodesPerTree[treeIdx]; ++nodeIdx)
    {
      const GPUForestNode& node = forestData[nodeIdx * GPUFOREST_NTREES
          + treeIdx];
      out << node.leftChildIdx << ' ' << node.leafIdx << ' ' << node.featureIdx
          << ' ' << std::setprecision(7) << node.featureThreshold << '\n';
    }
  }
}

size_t GPUForest::get_nb_trees() const
{
  return GPUFOREST_NTREES;
}

size_t GPUForest::get_nb_nodes_in_tree(size_t treeIdx) const
{
  if (treeIdx >= get_nb_trees())
    throw std::runtime_error("invalid treeIdx");

  return m_nbNodesPerTree[treeIdx];
}

size_t GPUForest::get_nb_leaves_in_tree(size_t treeIdx) const
{
  if (treeIdx >= get_nb_trees())
    throw std::runtime_error("invalid treeIdx");

  return m_nbLeavesPerTree[treeIdx];
}

//#################### SCOREFOREST INTEROP FUNCTIONS ####################
#ifdef WITH_SCOREFORESTS

GPUForest::GPUForest(const EnsembleLearner &pretrained_forest) :
    GPUForest()
{
  // Convert list of nodes into an appropriate image
  const int nTrees = pretrained_forest.GetNbTrees();
  const int maxNbNodes = pretrained_forest.GetMaxNbNodesInAnyLearner();

  if (nTrees != GPUFOREST_NTREES)
  {
    throw std::runtime_error(
        "Number of trees in the loaded forest different from the instantiation of GPUForest.");
  }

  // Create texture storing the nodes
  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_forestImage = mbf.make_image<GPUForestNode>(Vector2i(nTrees, maxNbNodes));
  m_forestImage->Clear();

  // Fill the nodes
  GPUForestNode *forestData = m_forestImage->GetData(MEMORYDEVICE_CPU);

  for (int treeIdx = 0; treeIdx < nTrees; ++treeIdx)
  {
    const Learner* tree = pretrained_forest.GetTree(treeIdx);
    const int nbNodes = tree->GetNbNodes();
    // Bug in scoreforest: Always returns 1 for loaded trees because
    // the base learner class does not store the leaves and DTBP does not perform the loading.
    // const int nbLeaves = tree->GetNbLeaves();

    m_nbNodesPerTree.push_back(nbNodes);
    // m_nbLeavesPerTree.push_back(nbLeaves);

    const int nbLeavesBefore = m_leafPredictions.size();
    // We set the first free entry to 1 since we reserve 0 for the root
    convert_node(tree, 0, treeIdx, nTrees, 0, 1, forestData);
    const int nbLeavesAfter = m_leafPredictions.size();

    std::cout << "Converted tree " << treeIdx << ", had " << nbNodes
        << " nodes and " << nbLeavesAfter - nbLeavesBefore << " leaves."
        << std::endl;
    std::cout << "Total number of leaves: " << m_leafPredictions.size()
        << std::endl;

    m_nbLeavesPerTree.push_back(nbLeavesAfter - nbLeavesBefore);
  }

  m_predictionsBlock = mbf.make_block<GPUForestPrediction>(
      m_leafPredictions.size());
  convert_predictions();

  // NOPs if we use the CPU only implementation
  m_forestImage->UpdateDeviceFromHost();
  m_predictionsBlock->UpdateDeviceFromHost();

  float meanShiftBandWidth = 0.1f;
  float cellLength = sqrt(meanShiftBandWidth * meanShiftBandWidth / 3) / 2.0f;
  float minStep = meanShiftBandWidth / 10.0f;

  m_ms3D = boost::make_shared<MeanShift3D>(meanShiftBandWidth, cellLength,
      minStep);

  {
    boost::timer::auto_cpu_timer t(6,
        "creating and clearing reservoirs: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
    m_leafReservoirs.reset(
        new GPUReservoir_CUDA(RESERVOIR_SIZE, m_predictionsBlock->dataSize));
  }
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

void GPUForest::convert_predictions()
{
  GPUForestPrediction *gpuPredictions = m_predictionsBlock->GetData(
      MEMORYDEVICE_CPU);

#pragma omp parallel for
  for (size_t leafIdx = 0; leafIdx < m_leafPredictions.size(); ++leafIdx)
  {
    const PredictionGaussianMean &currentPred = m_leafPredictions[leafIdx];

    // copy to sort modes by descending number of inliers so to keep only the best ones
    auto modes = currentPred._modes;
    std::sort(modes.begin(), modes.end(),
        [](const std::vector<PredictedGaussianMean> &a, const std::vector<PredictedGaussianMean> &b)
        { return a[0]._nbPoints > b[0]._nbPoints;});

    GPUForestPrediction &currentTargetPred = gpuPredictions[leafIdx];
    currentTargetPred.nbModes = 0; // Reset modes

    for (size_t modeIdx = 0;
        modeIdx < modes.size()
            && currentTargetPred.nbModes < GPUForestPrediction::MAX_MODES;
        ++modeIdx)
    {
      const auto &mode = modes[modeIdx];
      auto &targetMode = currentTargetPred.modes[currentTargetPred.nbModes++];

      // Not using _meanf and the others because the float variant sometimes seems not set..
      Eigen::Map<Eigen::Vector3f>(targetMode.position.v) = mode[0]._mean.cast<
          float>();
      Eigen::Map<Eigen::Matrix3f>(targetMode.positionInvCovariance.m) =
          mode[0]._inverseCovariance.cast<float>();
      targetMode.determinant = static_cast<float>(mode[0]._determinant);

      // Downcast the colour from float to uchar (loss of precision is acceptable)
      targetMode.colour.x = static_cast<uint8_t>(mode[1]._mean(0));
      targetMode.colour.y = static_cast<uint8_t>(mode[1]._mean(1));
      targetMode.colour.z = static_cast<uint8_t>(mode[1]._mean(2));

      targetMode.nbInliers = mode[0]._nbPoints;
    }
  }
}

#endif

}
