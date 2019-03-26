/**
 * grove: ScoreNetRelocaliser_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/cpu/ScoreNetRelocaliser_CPU.h"
using namespace ORUtils;
using namespace tvgutil;

#include "relocalisation/shared/ScoreNetRelocaliser_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreNetRelocaliser_CPU::ScoreNetRelocaliser_CPU(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace)
: ScoreNetRelocaliser(settings, settingsNamespace, DEVICE_CPU)
{}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreNetRelocaliser_CPU::set_bucket_predictions_for_keypoints(const BucketIndicesImage_CPtr& bucketIndices, ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = bucketIndices->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  const BucketIndices *bucketIndicesPtr = bucketIndices->GetData(MEMORYDEVICE_CPU);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CPU);
  const ScorePrediction *predictionsBlockPtr = m_relocaliserState->predictionsBlock->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      set_bucket_prediction_for_keypoint(x, y, imgSize, bucketIndicesPtr, predictionsBlockPtr, outputPredictionsPtr);
    }
  }
}

void ScoreNetRelocaliser_CPU::set_net_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const ScoreNetOutput_CPtr& scoreNetOutput,
                                                                ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = keypointsImage->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  const Keypoint3DColour *keypointsPtr = keypointsImage->GetData(MEMORYDEVICE_CPU);
  const float *scoreNetOutputPtr = scoreNetOutput->GetData(MEMORYDEVICE_CPU);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      set_net_prediction_for_keypoint(x, y, imgSize, keypointsPtr, scoreNetOutputPtr, outputPredictionsPtr);
    }
  }
}

}
