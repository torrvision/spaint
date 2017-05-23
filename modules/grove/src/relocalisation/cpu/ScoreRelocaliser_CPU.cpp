/**
 * grove: ScoreRelocaliser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/cpu/ScoreRelocaliser_CPU.h"

#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Utils/ITMLibSettings.h>
using namespace ITMLib;

#include <itmx/base/MemoryBlockFactory.h>
using namespace itmx;

#include "clustering/ExampleClustererFactory.h"
#include "features/FeatureCalculatorFactory.h"
#include "forests/DecisionForestFactory.h"
#include "ransac/RansacFactory.h"
#include "reservoirs/ExampleReservoirsFactory.h"

#include "relocalisation/shared/ScoreRelocaliser_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreRelocaliser_CPU::ScoreRelocaliser_CPU(const std::string &forestFilename) : ScoreRelocaliser(forestFilename)
{
  // Instantiate the sub-algorithms knowing that we are running on the GPU.

  // Features.
  m_featureCalculator = FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(ITMLibSettings::DEVICE_CPU);

  // LowLevelEngine.
  m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(ITMLibSettings::DEVICE_CPU));

  // Forest.
  m_scoreForest = DecisionForestFactory<DescriptorType, FOREST_TREE_COUNT>::make_forest(ITMLibSettings::DEVICE_CPU,
                                                                                        m_forestFilename);

  // These variables have to be set here, since they depend on the forest.
  m_reservoirsCount = m_scoreForest->get_nb_leaves();
  m_predictionsBlock = MemoryBlockFactory::instance().make_block<ScorePrediction>(m_reservoirsCount);

  // Reservoirs.
  m_exampleReservoirs = ExampleReservoirsFactory<ExampleType>::make_reservoirs(
      ITMLibSettings::DEVICE_CPU, m_reservoirsCapacity, m_reservoirsCount, m_rngSeed);

  // Clustering.
  m_exampleClusterer = ExampleClustererFactory<ExampleType, ClusterType, PredictionType::MAX_CLUSTERS>::make_clusterer(
      ITMLibSettings::DEVICE_CPU, m_clustererSigma, m_clustererTau, m_maxClusterCount, m_minClusterSize);

  // P-RANSAC.
  m_preemptiveRansac = RansacFactory::make_preemptive_ransac(ITMLibSettings::DEVICE_CPU);

  // Clear internal state.
  reset();
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

ScorePrediction ScoreRelocaliser_CPU::get_raw_prediction(uint32_t treeIdx, uint32_t leafIdx) const
{
  if (treeIdx >= m_scoreForest->get_nb_trees() || leafIdx >= m_scoreForest->get_nb_leaves_in_tree(treeIdx))
  {
    throw std::invalid_argument("Invalid tree or leaf index.");
  }

  return m_predictionsBlock->GetElement(leafIdx * m_scoreForest->get_nb_trees() + treeIdx, MEMORYDEVICE_CPU);
}

//#################### PROTECTED VIRTUAL MEMBER FUNCTIONS ####################

void ScoreRelocaliser_CPU::get_predictions_for_leaves(const LeafIndicesImage_CPtr &leafIndices,
                                                      const ScorePredictionsBlock_CPtr &leafPredictions,
                                                      ScorePredictionsImage_Ptr &outputPredictions) const
{
  const Vector2i imgSize = leafIndices->noDims;
  const LeafIndices *leafIndicesData = leafIndices->GetData(MEMORYDEVICE_CPU);

  const ScorePrediction *leafPredictionsData = leafPredictions->GetData(MEMORYDEVICE_CPU);

  // NOP after the first time.
  outputPredictions->ChangeDims(imgSize);
  ScorePrediction *outPredictionsData = outputPredictions->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < imgSize.y; ++y)
  {
    for (int x = 0; x < imgSize.x; ++x)
    {
      get_prediction_for_leaf_shared(
          leafPredictionsData, leafIndicesData, outPredictionsData, imgSize, m_maxClusterCount, x, y);
    }
  }
}

} // namespace grove
