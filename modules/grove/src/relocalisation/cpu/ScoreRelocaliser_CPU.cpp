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

ScoreRelocaliser_CPU::ScoreRelocaliser_CPU(const tvgutil::SettingsContainer_CPtr& settings, const std::string& forestFilename)
  : ScoreRelocaliser(settings, forestFilename)
{
  // Instantiate the sub-algorithms knowing that we are running on the GPU.

  // Features.
  m_featureCalculator = FeatureCalculatorFactory::make_da_rgbd_patch_feature_calculator(ITMLibSettings::DEVICE_CPU);

  // LowLevelEngine.
  m_lowLevelEngine.reset(ITMLowLevelEngineFactory::MakeLowLevelEngine(ITMLibSettings::DEVICE_CPU));

  // Forest.
  m_scoreForest = DecisionForestFactory<DescriptorType, FOREST_TREE_COUNT>::make_forest(m_forestFilename, ITMLibSettings::DEVICE_CPU);

  // These variables have to be set here, since they depend on the forest.
  m_reservoirsCount = m_scoreForest->get_nb_leaves();


  // Clustering.
  m_exampleClusterer = ExampleClustererFactory<ExampleType, ClusterType, PredictionType::Capacity>::make_clusterer(
      ITMLibSettings::DEVICE_CPU, m_clustererSigma, m_clustererTau, m_maxClusterCount, m_minClusterSize);

  // P-RANSAC.
  m_preemptiveRansac = RansacFactory::make_preemptive_ransac(ITMLibSettings::DEVICE_CPU, m_settings);

  // Clear internal state.
  ScoreRelocaliser_CPU::reset();
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

ScorePrediction ScoreRelocaliser_CPU::get_raw_prediction(uint32_t treeIdx, uint32_t leafIdx) const
{
  if (treeIdx >= m_scoreForest->get_nb_trees() || leafIdx >= m_scoreForest->get_nb_leaves_in_tree(treeIdx))
  {
    throw std::invalid_argument("Invalid tree or leaf index.");
  }

  return m_relocaliserState->predictionsBlock->GetElement(leafIdx * m_scoreForest->get_nb_trees() + treeIdx, MEMORYDEVICE_CPU);
}

std::vector<Keypoint3DColour> ScoreRelocaliser_CPU::get_reservoir_contents(uint32_t treeIdx, uint32_t leafIdx) const
{
  if (treeIdx >= m_scoreForest->get_nb_trees() || leafIdx >= m_scoreForest->get_nb_leaves_in_tree(treeIdx))
  {
    throw std::invalid_argument("Invalid tree or leaf index.");
  }

  const uint32_t linearReservoirIdx = leafIdx * m_scoreForest->get_nb_trees() + treeIdx;
  const uint32_t currentReservoirSize = m_relocaliserState->exampleReservoirs->get_reservoir_sizes()->GetElement(linearReservoirIdx, MEMORYDEVICE_CPU);
  const uint32_t reservoirCapacity = m_relocaliserState->exampleReservoirs->get_reservoir_capacity();

  std::vector<Keypoint3DColour> reservoirContents;
  reservoirContents.reserve(currentReservoirSize);

  const Keypoint3DColour *reservoirData = m_relocaliserState->exampleReservoirs->get_reservoirs()->GetData(MEMORYDEVICE_CPU);

  for(uint32_t i = 0; i < currentReservoirSize; ++i)
  {
    reservoirContents.push_back(reservoirData[linearReservoirIdx * reservoirCapacity + i]);
  }

  return reservoirContents;
}

void ScoreRelocaliser_CPU::reset()
{
  // Setup the reservoirs if they haven't been allocated yet.
  if(!m_relocaliserState->exampleReservoirs)
    m_relocaliserState->exampleReservoirs = ExampleReservoirsFactory<ExampleType>::make_reservoirs(m_reservoirsCount, m_reservoirCapacity, ITMLibSettings::DEVICE_CPU, m_rngSeed);

  // Setup the predictions block.
  if(!m_relocaliserState->predictionsBlock)
    m_relocaliserState->predictionsBlock = MemoryBlockFactory::instance().make_block<ScorePrediction>(m_reservoirsCount);

  ScoreRelocaliser::reset();
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
