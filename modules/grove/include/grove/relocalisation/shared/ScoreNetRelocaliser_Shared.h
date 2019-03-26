/**
 * grove: ScoreNetRelocaliser_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_GROVE_SCORENETRELOCALISER_SHARED
#define H_GROVE_SCORENETRELOCALISER_SHARED

#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief Sets a SCoRe prediction for the specified keypoint that contains all of the clusters in the keypoint's bucket (example reservoir).
 *
 * \param x                 The x coordinate of the keypoint.
 * \param y                 The y coordinate of the keypoint.
 * \param imgSize           The dimensions of the bucketIndices and outputPredictions images.
 * \param bucketIndices     A pointer to an image containing the bucket (example reservoir) indices associated with the keypoints.
 * \param predictionsBlock  A pointer to the storage area holding all of the SCoRe predictions associated with the example reservoirs.
 * \param outputPredictions A pointer to the image in which to store the output SCoRe predictions.
 */
_CPU_AND_GPU_CODE_
inline void set_bucket_prediction_for_keypoint(int x, int y, const Vector2i& imgSize, const ORUtils::VectorX<int,1> *bucketIndices,
                                               const ScorePrediction *predictionsBlock, ScorePrediction *outputPredictions)
{
  const int offset = y * imgSize.x + x;
  outputPredictions[offset] = predictionsBlock[bucketIndices[offset][0]];
}

/**
 * \brief Sets a SCoRe prediction for the specified keypoint that contains a single cluster consisting of the world space point predicted by the network for the keypoint.
 *
 * \param x                 The x coordinate of the keypoint.
 * \param y                 The y coordinate of the keypoint.
 * \param imgSize           The dimensions of the keypoints and outputPredictions images.
 * \param keypoints         A pointer to an image containing the keypoints extracted from the RGB-D image.
 * \param scoreNetOutput    A pointer to a memory block containing the output tensor produced by the SCoRe network.
 * \param outputPredictions A pointer to the image in which to store the output SCoRe predictions.
 */
_CPU_AND_GPU_CODE_
inline void set_net_prediction_for_keypoint(int x, int y, const Vector2i& imgSize, const Keypoint3DColour *keypoints, const float *scoreNetOutput, ScorePrediction *outputPredictions)
{
  const int planeOffset = imgSize.x * imgSize.y;
  const int pixelOffset = y * imgSize.x + x;
  const Keypoint3DColour& keypoint = keypoints[pixelOffset];
  ScorePrediction& outputPrediction = outputPredictions[pixelOffset];

  outputPrediction.size = 1;

  Keypoint3DColourCluster& outputCluster = outputPrediction.elts[0];
  outputCluster.position = Vector3f(scoreNetOutput[pixelOffset], scoreNetOutput[planeOffset + pixelOffset], scoreNetOutput[2 * planeOffset + pixelOffset]);
  outputCluster.colour = keypoint.colour;
  outputCluster.nbInliers = 1;
  outputCluster.positionInvCovariance.setIdentity();
  outputCluster.determinant = 1.0f;
}

}

#endif
