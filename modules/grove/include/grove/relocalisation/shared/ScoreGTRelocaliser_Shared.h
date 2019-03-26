/**
 * grove: ScoreGTRelocaliser_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCOREGTRELOCALISER_SHARED
#define H_GROVE_SCOREGTRELOCALISER_SHARED

#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief Sets a SCoRe prediction for the specified keypoint that contains a single cluster consisting of the ground truth position of the keypoint in world space.
 *
 * \param x                 The x coordinate of the keypoint.
 * \param y                 The y coordinate of the keypoint.
 * \param imgSize           The dimensions of the keypoints and outputPredictions images.
 * \param keypoints         A pointer to an image containing the keypoints extracted from the RGB-D image.
 * \param cameraToWorld     The ground truth transformation from camera space to world space.
 * \param outputPredictions A pointer to the image in which to store the output SCoRe predictions.
 */
_CPU_AND_GPU_CODE_
inline void set_ground_truth_prediction_for_keypoint(int x, int y, const Vector2i& imgSize, const Keypoint3DColour *keypoints, const Matrix4f& cameraToWorld, ScorePrediction *outputPredictions)
{
  const int offset = y * imgSize.x + x;
  const Keypoint3DColour& keypoint = keypoints[offset];
  ScorePrediction& outputPrediction = outputPredictions[offset];

  outputPrediction.size = 1;

  Keypoint3DColourCluster& outputCluster = outputPrediction.elts[0];
  outputCluster.position = cameraToWorld * keypoint.position;
  outputCluster.colour = keypoint.colour;
  outputCluster.nbInliers = 1;
  outputCluster.positionInvCovariance.setIdentity();
  outputCluster.determinant = 1.0f;
}

}

#endif
