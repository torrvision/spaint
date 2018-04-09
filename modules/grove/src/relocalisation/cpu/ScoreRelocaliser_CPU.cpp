/**
 * grove: ScoreRelocaliser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/cpu/ScoreRelocaliser_CPU.h"
using namespace tvgutil;

#include "relocalisation/shared/ScoreRelocaliser_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreRelocaliser_CPU::ScoreRelocaliser_CPU(const SettingsContainer_CPtr& settings, const std::string& forestFilename)
: ScoreRelocaliser(settings, DEVICE_CPU, forestFilename)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::vector<Keypoint3DColour> ScoreRelocaliser_CPU::get_reservoir_contents(uint32_t treeIdx, uint32_t leafIdx) const
{
  if(treeIdx >= m_scoreForest->get_nb_trees() || leafIdx >= m_scoreForest->get_nb_leaves_in_tree(treeIdx))
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

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreRelocaliser_CPU::get_predictions_for_leaves(const LeafIndicesImage_CPtr& leafIndices, const ScorePredictionsMemoryBlock_CPtr& leafPredictions,
                                                      ScorePredictionsImage_Ptr& outputPredictions) const
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
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      get_prediction_for_leaf_shared(leafPredictionsData, leafIndicesData, outPredictionsData, imgSize, m_maxClusterCount, x, y);
    }
  }
}

}
