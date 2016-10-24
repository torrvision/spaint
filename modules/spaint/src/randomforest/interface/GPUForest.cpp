/**
 * spaint: GPUForest.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/GPUForest.h"

#include <boost/make_shared.hpp>
#include <boost/timer/timer.hpp>

#include "util/MemoryBlockFactory.h"

#include "randomforest/cuda/GPUReservoir_CUDA.h"

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

  m_predictionsBlock = mbf.make_block<GPUForestPrediction>(
      m_leafPredictions.size());
  convert_predictions();

  // NOPs if we use the CPU only implementation
  m_forestImage->UpdateDeviceFromHost();
  m_predictionsBlock->UpdateDeviceFromHost();

  // Allocate the image that will store the leaf indices (dummy size, will be resized as needed)
  m_leafImage = mbf.make_image<LeafIndices>(Vector2i(0, 0));

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

void GPUForest::reset_predictions()
{
  m_predictionsBlock->Clear(); // Setting nbModes to 0 for each prediction would be enough.
  m_predictionsBlock->UpdateDeviceFromHost();
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
  }

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "add examples to reservoirs: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_leafReservoirs->add_examples(features, m_leafImage);
  }

#if 1

  // Perform meanshift on each reservoir, just to see if the thing works
  static int callCount = 0;

  if (callCount++ != 999)
    return; // only at the last

//#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6,
      "meanshift: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//#endif

  m_leafReservoirs->get_reservoirs()->UpdateHostFromDevice();
  const GPUReservoir::ExampleType *reservoirs =
      m_leafReservoirs->get_reservoirs()->GetData(MEMORYDEVICE_CPU);

  m_leafReservoirs->get_reservoirs_size()->UpdateHostFromDevice();
  const int *reservoirsSize = m_leafReservoirs->get_reservoirs_size()->GetData(
      MEMORYDEVICE_CPU);

  const int nbLeaves = m_predictionsBlock->dataSize; // same as the number of reservoirs
  const int reservoirCapacity = m_leafReservoirs->get_capacity();

//  const int totalEntries = nbLeaves * reservoirCapacity;
//  int occupiedEntries = 0;
//
//  for (int i = 0; i < nbLeaves; ++i)
//  {
//    int x = res[i];
//    if (x < 0 || x >= 1000)
//    {
//      std::cout << "Res: " << i << ": " << x << std::endl;
//    }
//
//    occupiedEntries += x;
//  }
//
//  std::cout << "Occupied entries: " << occupiedEntries << "/" << totalEntries
//      << std::endl;

  GPUForestPrediction *predictionsData = m_predictionsBlock->GetData(
      MEMORYDEVICE_CPU);

  int smallModes = 0;

  for (size_t leafIdx = 0; leafIdx < nbLeaves; ++leafIdx)
  {
    GPUForestPrediction &currentPrediction = predictionsData[leafIdx];
    currentPrediction.nbModes = 0;

    const int nbSamples = reservoirsSize[leafIdx];
    const PositionColourExample *samples = &reservoirs[leafIdx
        * reservoirCapacity];

    if (nbSamples == 0)
      continue;

    // Convert samples in a format that scoreforest likes
    std::vector<IOData> ioDataStoragePos, ioDataStorageCol; // To destruct cleanly
    ioDataStoragePos.resize(nbSamples);
    ioDataStorageCol.resize(nbSamples);

    std::vector<IOData *> ioDataForMeanshiftPos, ioDataForMeanshiftCol;
    ioDataForMeanshiftPos.reserve(nbSamples);
    ioDataForMeanshiftCol.reserve(nbSamples);

    for (size_t i = 0; i < nbSamples; ++i)
    {
      ioDataStoragePos[i].Init(3);
      ioDataStoragePos[i]._data[0] = samples[i].position.x;
      ioDataStoragePos[i]._data[1] = samples[i].position.y;
      ioDataStoragePos[i]._data[2] = samples[i].position.z;

      ioDataStorageCol[i].Init(3);
      ioDataStorageCol[i]._data[0] = samples[i].colour.x;
      ioDataStorageCol[i]._data[1] = samples[i].colour.y;
      ioDataStorageCol[i]._data[2] = samples[i].colour.z;
    }

    for (size_t i = 0; i < nbSamples; ++i)
    {
      ioDataForMeanshiftPos.push_back(&ioDataStoragePos[i]);
      ioDataForMeanshiftCol.push_back(&ioDataStorageCol[i]);
    }

    std::vector<std::pair<CellCorner, std::vector<int>>>msResult;
//    std::cout << "For leaf " << leafIdx << " computing MS for " << nbSamples
//        << " samples." << std::endl;
    {
      boost::timer::auto_cpu_timer t(6,
          "MS: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
      msResult = m_ms3D->Process(ioDataForMeanshiftPos);
    }

    std::cout << "For leaf " << leafIdx << " computed MS for " << nbSamples
        << " samples, found " << msResult.size() << " modes.\n";

    // Sort mode by descending inliers
    std::sort(msResult.begin(), msResult.end(),
        [] (const std::pair<CellCorner, std::vector<int>> &a, const std::pair<CellCorner, std::vector<int>> &b)
        { return a.second.size() > b.second.size();});

    // Keep at most MAX_MODES modes
    const int nbModes = std::min<size_t>(msResult.size(),
        GPUForestPrediction::MAX_MODES);

    // convert them into the correct format
    for (int modeIdx = 0; modeIdx < nbModes; ++modeIdx)
    {
      const std::vector<int>& inliers = msResult[modeIdx].second;
      GPUForestMode &outMode = currentPrediction.modes[modeIdx];

      // TODO: possibly trigger this after a while
//      if ((int) inliers.size() < _minClusterSize)
//      {
//        continue;
//      }

      outMode.nbInliers = inliers.size();
      if (outMode.nbInliers == 0)
        throw std::runtime_error("mode with no inliers");

      if (outMode.nbInliers < 20)
      {
        smallModes++;
        continue;
      }

      // compute the gaussian for the position (TODO: just need mean and covariance (+ inverse), could be improved)
      {
        GaussianAggregator ga(3);
        for (size_t s = 0; s < inliers.size(); ++s)
        {
          ga.Aggregate(ioDataForMeanshiftPos[inliers[s]]);
        }

        Eigen::Map<Eigen::Vector3f>(outMode.position.v) = ga.GetMean().cast<
            float>();

        Eigen::MatrixXd cov = ga.GetCovariance();
        Eigen::Map<Eigen::Matrix3f>(outMode.positionInvCovariance.m) =
            cov.inverse().cast<float>();

        outMode.determinant = cov.determinant();
      }

      // Compute the gaussian for the colour (TODO: just computing the mean would be enough)
      {
        GaussianAggregator ga(3);
        for (size_t s = 0; s < inliers.size(); ++s)
        {
          ga.Aggregate(ioDataForMeanshiftCol[inliers[s]]);
        }

        Eigen::Vector3d colMode = ga.GetMean();
        outMode.colour.x = colMode(0);
        outMode.colour.y = colMode(1);
        outMode.colour.z = colMode(2);
      }

      currentPrediction.nbModes++;

//      std::cout << "Mode " << modeIdx << ", pos: " << outMode.position
//          << " - col: " << outMode.colour.toFloat() << " - inliers: "
//          << outMode.nbInliers << std::endl;
    }

//    break;
  }

  std::cout << "Modes with few inliers: " << smallModes << std::endl;

  // Copy new predictions on the GPU
  m_predictionsBlock->UpdateDeviceFromHost();
#endif
}

int GPUForestPrediction::get_best_mode(const Vector3f &v) const
{
  float energy;
  return get_best_mode_and_energy(v, energy);
}

int GPUForestPrediction::get_best_mode_and_energy(const Vector3f &v,
    float &maxScore) const
{
  static const float exponent = powf(2.0f * M_PI, 3);

  int argmax = -1;
  maxScore = std::numeric_limits<float>::lowest();

  for (int m = 0; m < nbModes; ++m)
  {
    const float nbPts = static_cast<float>(modes[m].nbInliers);
    const Vector3f diff = v - modes[m].position;

    const float normalization = 1.0 / sqrtf(modes[m].determinant * exponent);
    // This is the textbook implementation of Mahalanobis distance
    // Helpers::MahalanobisSquared3x3 used in the original code seems wrong
    const float mahalanobisSq = dot(diff,
        modes[m].positionInvCovariance * diff);
    const float descriptiveStatistics = expf(-0.5f * mahalanobisSq);
    const float evalGaussian = normalization * descriptiveStatistics;
    const float score = nbPts * evalGaussian;

    if (score > maxScore)
    {
      maxScore = score;
      argmax = m;
    }
  }

  return argmax;
}

}
