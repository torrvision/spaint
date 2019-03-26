/**
 * grove: ScoreGTRelocaliser_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/cuda/ScoreGTRelocaliser_CUDA.h"
using namespace ORUtils;
using namespace tvgutil;

#include "relocalisation/shared/ScoreGTRelocaliser_Shared.h"

namespace grove {

//#################### CUDA KERNELS ####################

__global__ void ck_set_ground_truth_predictions_for_keypoints(const Keypoint3DColour *keypoints, Matrix4f cameraToWorld, Vector2i imgSize, ScorePrediction *outputPredictions)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;

  if(x < imgSize.x && y < imgSize.y)
  {
    set_ground_truth_prediction_for_keypoint(x, y, imgSize, keypoints, cameraToWorld, outputPredictions);
  }
}

//#################### CONSTRUCTORS ####################

ScoreGTRelocaliser_CUDA::ScoreGTRelocaliser_CUDA(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace)
: ScoreGTRelocaliser(settings, settingsNamespace, DEVICE_CUDA)
{}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreGTRelocaliser_CUDA::set_ground_truth_predictions_for_keypoints(const Keypoint3DColourImage_CPtr& keypointsImage, const Matrix4f& cameraToWorld,
                                                                         ScorePredictionsImage_Ptr& outputPredictions) const
{
  const Vector2i imgSize = keypointsImage->noDims;

  // Make sure that the output predictions image has the right size (this is a no-op after the first time).
  outputPredictions->ChangeDims(imgSize);

  const Keypoint3DColour *keypointsPtr = keypointsImage->GetData(MEMORYDEVICE_CUDA);
  ScorePrediction *outputPredictionsPtr = outputPredictions->GetData(MEMORYDEVICE_CUDA);

  const dim3 blockSize(32, 32);
  const dim3 gridSize((imgSize.x + blockSize.x - 1) / blockSize.x, (imgSize.y + blockSize.y - 1) / blockSize.y);

  ck_set_ground_truth_predictions_for_keypoints<<<gridSize, blockSize>>>(
    keypointsPtr, cameraToWorld, imgSize, outputPredictionsPtr
  );
  ORcudaKernelCheck;
}

}
