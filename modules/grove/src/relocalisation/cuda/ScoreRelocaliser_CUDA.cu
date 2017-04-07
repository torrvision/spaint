/**
 * grove: ScoreRelocaliser_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/cuda/ScoreRelocaliser_CUDA.h"

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
using ITMLib::ITMLowLevelEngineFactory;
#include <ITMLib/Utils/ITMLibSettings.h>
using ITMLib::ITMLibSettings;

#include <itmx/MemoryBlockFactory.h>
using itmx::MemoryBlockFactory;

#include "clustering/ExampleClustererFactory.h"
#include "features/FeatureCalculatorFactory.h"
#include "forests/DecisionForestFactory.h"
#include "ransac/RansacFactory.h"
#include "reservoirs/ExampleReservoirsFactory.h"

#include "relocalisation/shared/ScoreRelocaliser_Shared.h"

namespace grove {

template<int TREE_COUNT>
__global__ void ck_get_predictions(
    const Prediction3DColour* leafPredictions,
    const ORUtils::VectorX<int, TREE_COUNT>* leafIndices,
    Prediction3DColour* outPredictions,
    Vector2i imgSize)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  get_prediction_for_leaf_shared(leafPredictions, leafIndices, outPredictions, imgSize, x, y);
}

ScoreRelocaliser_CUDA::ScoreRelocaliser_CUDA(const std::string &forestFilename)
  : ScoreRelocaliser(forestFilename)
{
  m_featureCalculator = FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(ITMLibSettings::DEVICE_CUDA);

  m_scoreForest = DecisionForestFactory<DescriptorType, TREE_COUNT>::make_forest(ITMLibSettings::DEVICE_CUDA, m_forestFilename);

  // These variables have to be set here, since they depend on the forest.
  m_reservoirsCount = m_scoreForest->get_nb_leaves();
  m_predictionsBlock = MemoryBlockFactory::instance().make_block<ClusterType>(m_reservoirsCount);

  m_exampleReservoirs = ExampleReservoirsFactory<ExampleType>::make_reservoirs(ITMLibSettings::DEVICE_CUDA, m_reservoirsCapacity, m_reservoirsCount, m_rngSeed);
  m_exampleClusterer = ExampleClustererFactory<ExampleType, ClusterType>::make_clusterer(ITMLibSettings::DEVICE_CUDA, m_clustererSigma, m_clustererTau, m_maxClusterCount, m_minClusterSize);

  // RANSAC is forced to the CPU implementation for now, the CUDA one is slower.
//  m_preemptiveRansac = RansacFactory::make_preemptive_ransac(ITMLibSettings::DEVICE_CPU);
  m_preemptiveRansac = RansacFactory::make_preemptive_ransac(ITMLibSettings::DEVICE_CUDA);

  m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(ITMLibSettings::DEVICE_CUDA));

  // Clear state
  reset();
}

void ScoreRelocaliser_CUDA::get_predictions_for_leaves(
    const LeafIndicesImage_CPtr &leafIndices,
    const ScorePredictionsBlock_CPtr &leafPredictions,
    ScorePredictionsImage_Ptr &outputPredictions) const
{
  const Vector2i imgSize = leafIndices->noDims;
  const LeafIndices* leafIndicesData = leafIndices->GetData(MEMORYDEVICE_CUDA);

  // Leaf predictions
  const Prediction3DColour *leafPredictionsData = leafPredictions->GetData(MEMORYDEVICE_CUDA);

  // No-op after the first time.
  outputPredictions->ChangeDims(imgSize);
  Prediction3DColour *outPredictionsData = outputPredictions->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x, (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_get_predictions<<<gridSize,blockSize>>>(leafPredictionsData, leafIndicesData, outPredictionsData, imgSize);
  ORcudaKernelCheck;
}

}
