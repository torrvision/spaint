/**
 * grove: ScoreRelocaliser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "relocalisation/cpu/ScoreRelocaliser_CPU.h"
using namespace tvgutil;

#include "relocalisation/shared/ScoreRelocaliser_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreRelocaliser_CPU::ScoreRelocaliser_CPU(const std::string& forestFilename, const SettingsContainer_CPtr& settings)
: ScoreRelocaliser(forestFilename, settings, DEVICE_CPU)
{}

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
