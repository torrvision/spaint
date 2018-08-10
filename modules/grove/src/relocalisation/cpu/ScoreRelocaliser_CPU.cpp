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

uint32_t ScoreRelocaliser_CPU::count_valid_depths(const ORFloatImage *depthImage) const
{
  uint32_t validDepths = 0;

  const int imgArea = depthImage->noDims.width * depthImage->noDims.height;
  const float *depths = depthImage->GetData(MEMORYDEVICE_CPU);

  // Count the number of pixels having a valid (positive) depth measurement. In parallel if possible.
#ifdef WITH_OPENMP
  #pragma omp parallel for reduction(+:validDepths)
#endif
  for(int i = 0; i < imgArea; ++i)
  {
    validDepths += depths[i] > 0.0f;
  }

  return validDepths;
}

void ScoreRelocaliser_CPU::merge_predictions_for_keypoints(const LeafIndicesImage_CPtr& leafIndices, ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = leafIndices->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  const LeafIndices *leafIndicesPtr = leafIndices->GetData(MEMORYDEVICE_CPU);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CPU);
  const ScorePrediction *predictionsBlockPtr = m_relocaliserState->predictionsBlock->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      merge_predictions_for_keypoint(x, y, leafIndicesPtr, predictionsBlockPtr, imgSize, m_maxClusterCount, outputPredictionsPtr);
    }
  }
}

}
