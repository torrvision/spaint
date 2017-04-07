/**
 * grove: ScoreRelocaliser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/cpu/ScoreRelocaliser_CPU.h"

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
using ITMLib::ITMLowLevelEngineFactory;
#include <ITMLib/Utils/ITMLibSettings.h>
using ITMLib::ITMLibSettings;

#include <itmx/MemoryBlockFactory.h>
using namespace itmx;

#include "clustering/ExampleClustererFactory.h"
#include "features/FeatureCalculatorFactory.h"
#include "forests/DecisionForestFactory.h"
#include "ransac/RansacFactory.h"
#include "reservoirs/ExampleReservoirsFactory.h"

#include "relocalisation/shared/ScoreRelocaliser_Shared.h"

namespace grove {

ScoreRelocaliser_CPU::ScoreRelocaliser_CPU(const std::string &forestFilename)
  : ScoreRelocaliser(forestFilename)
{
  m_featureCalculator = FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(ITMLibSettings::DEVICE_CPU);

  m_scoreForest = DecisionForestFactory<DescriptorType, TREE_COUNT>::make_forest(ITMLibSettings::DEVICE_CPU, m_forestFilename);

  // These variables have to be set here, since they depend on the forest.
  m_reservoirsCount = m_scoreForest->get_nb_leaves();
  m_predictionsBlock = MemoryBlockFactory::instance().make_block<ClusterType>(m_reservoirsCount);

  m_exampleReservoirs = ExampleReservoirsFactory<ExampleType>::make_reservoirs(ITMLibSettings::DEVICE_CPU, m_reservoirsCapacity, m_reservoirsCount, m_rngSeed);
  m_exampleClusterer = ExampleClustererFactory<ExampleType, ClusterType>::make_clusterer(ITMLibSettings::DEVICE_CPU, m_clustererSigma, m_clustererTau, m_maxClusterCount, m_minClusterSize);
  m_preemptiveRansac = RansacFactory::make_preemptive_ransac(ITMLibSettings::DEVICE_CPU);

  m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(ITMLibSettings::DEVICE_CPU));

  // Clear state
  reset();
}

void ScoreRelocaliser_CPU::get_predictions_for_leaves(
    const LeafIndicesImage_CPtr &leafIndices,
    const ScorePredictionsBlock_CPtr &leafPredictions,
    ScorePredictionsImage_Ptr &outputPredictions) const
{
  const Vector2i imgSize = leafIndices->noDims;
  const LeafIndices* leafIndicesData = leafIndices->GetData(MEMORYDEVICE_CPU);

  // Leaf predictions
  const Prediction3DColour *leafPredictionsData = leafPredictions->GetData(MEMORYDEVICE_CPU);

  // No-op after the first time.
  outputPredictions->ChangeDims(imgSize);
  Prediction3DColour *outPredictionsData = outputPredictions->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      get_prediction_for_leaf_shared(leafPredictionsData, leafIndicesData, outPredictionsData, imgSize, x, y);
    }
  }
}

}
