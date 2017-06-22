/**
 * grove: ScorePrediction.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCOREPREDICTION
#define H_GROVE_SCOREPREDICTION

#include <ORUtils/PlatformIndependence.h>

#include <itmx/base/ITMImagePtrTypes.h>

#include "../clustering/base/ClusterContainer.h"
#include "Mode3DColour.h"

namespace grove {

//#################### TYPEDEFS ####################

typedef ClusterContainer<Mode3DColour, 50> ScorePrediction;

typedef ORUtils::MemoryBlock<ScorePrediction> ScorePredictionsBlock;
typedef boost::shared_ptr<ScorePredictionsBlock> ScorePredictionsBlock_Ptr;
typedef boost::shared_ptr<const ScorePredictionsBlock> ScorePredictionsBlock_CPtr;

typedef ORUtils::Image<ScorePrediction> ScorePredictionsImage;
typedef boost::shared_ptr<ScorePredictionsImage> ScorePredictionsImage_Ptr;
typedef boost::shared_ptr<const ScorePredictionsImage> ScorePredictionsImage_CPtr;

//#################### FREE FUNCTIONS ####################

/**
 * \brief Given a ScorePrediction and a 3D point, find the closest 3D Mode according to the Mahalanobis distance.
 *
 * \param prediction A ScorePrediction
 * \param v          A 3D point in world coordinates.
 * \param maxScore   Will contain the energy associated to the closest mode.
 *
 * \return The index of the closest modal cluster. -1 if nbModes is 0.
 */
_CPU_AND_GPU_CODE_
inline int score_prediction_get_best_mode_and_energy(const ScorePrediction &prediction, const Vector3f &v, float &maxScore)
{
  const float exponent = powf(2.0f * static_cast<float>(M_PI), 3);

  int argmax = -1;
  maxScore = -1.f; // If set to 0 the comparison later fails for very small values
  //    maxScore = 0.0f;

  // Iterate over all the modal clusters stored in the struct.
  for (int m = 0; m < prediction.size; ++m)
  {
    const Mode3DColour &currentMode = prediction.clusters[m];

    const Vector3f diff = v - currentMode.position;

    // This is the textbook implementation of Mahalanobis distance
    // Helpers::MahalanobisSquared3x3 used in the scoreforests code seems wrong.
    const float mahalanobisSq = dot(diff, currentMode.positionInvCovariance * diff);
    const float descriptiveStatistics = expf(-0.5f * mahalanobisSq);

    const float normalization = 1.0f / sqrtf(currentMode.determinant * exponent);
    const float evalGaussian = normalization * descriptiveStatistics;

    const float nbPts = static_cast<float>(currentMode.nbInliers);
    const float score = nbPts * evalGaussian;

    // If the point is "closer" to the center of the anisotropic gaussian associated to this mode, store its index.
    if (score > maxScore)
    {
      maxScore = score;
      argmax = m;
    }
  }

  return argmax;
}

/**
 * \brief Given a ScorePrediction and a 3D point, find the closest 3D Mode according to the Mahalanobis distance.
 *
 * \param prediction A ScorePrediction
 * \param v          A 3D point in world coordinates.
 *
 * \return The index of the closest modal cluster. -1 if nbModes is 0.
 */
_CPU_AND_GPU_CODE_
inline int score_prediction_get_best_mode(const ScorePrediction &prediction, const Vector3f &v)
{
  float energy;
  return score_prediction_get_best_mode_and_energy(prediction, v, energy);
}

} // namespace grove

#endif // H_GROVE_SCOREPREDICTION
