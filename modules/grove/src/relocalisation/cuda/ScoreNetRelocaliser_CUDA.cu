/**
 * grove: ScoreNetRelocaliser_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/cuda/ScoreNetRelocaliser_CUDA.h"
using namespace ORUtils;
using namespace tvgutil;

#include "relocalisation/shared/ScoreNetRelocaliser_Shared.h"

namespace grove {

//#################### CUDA KERNELS ####################

__global__ void ck_set_bucket_predictions_for_keypoints(const ORUtils::VectorX<int,1> *bucketIndices, const ScorePrediction *predictionsBlock, Vector2i imgSize,
                                                        ScorePrediction *outputPredictions)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if(x < imgSize.x && y < imgSize.y)
  {
    set_bucket_prediction_for_keypoint(x, y, imgSize, bucketIndices, predictionsBlock, outputPredictions);
  }
}

__global__ void ck_set_net_predictions_for_keypoints(const Keypoint3DColour *keypoints, const float *scoreNetOutput, Vector2i imgSize, ScorePrediction *outputPredictions)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if(x < imgSize.x && y < imgSize.y)
  {
    set_net_prediction_for_keypoint(x, y, imgSize, keypoints, scoreNetOutput, outputPredictions);
  }
}

//#################### CONSTRUCTORS ####################

ScoreNetRelocaliser_CUDA::ScoreNetRelocaliser_CUDA(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace)
: ScoreNetRelocaliser(settings, settingsNamespace, DEVICE_CUDA)
{}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreNetRelocaliser_CUDA::set_bucket_predictions_for_keypoints(const BucketIndicesImage_CPtr& bucketIndices, ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = bucketIndices->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  const BucketIndices *bucketIndicesPtr = bucketIndices->GetData(MEMORYDEVICE_CUDA);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CUDA);
  const ScorePrediction *predictionsBlockPtr = m_relocaliserState->predictionsBlock->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x, (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_set_bucket_predictions_for_keypoints<<<gridSize, blockSize>>>(
    bucketIndicesPtr, predictionsBlockPtr, imgSize, outputPredictionsPtr
  );
  ORcudaKernelCheck;
}

void ScoreNetRelocaliser_CUDA::set_net_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const ScoreNetOutput_CPtr& scoreNetOutput,
                                                                 ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = keypointsImage->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  const Keypoint3DColour *keypointsPtr = keypointsImage->GetData(MEMORYDEVICE_CUDA);
  const float *scoreNetOutputPtr = scoreNetOutput->GetData(MEMORYDEVICE_CUDA);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x, (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_set_net_predictions_for_keypoints<<<gridSize, blockSize>>>(
    keypointsPtr, scoreNetOutputPtr, imgSize, outputPredictionsPtr
  );
  ORcudaKernelCheck;
}

}
