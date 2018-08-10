/**
 * grove: ScorePrediction.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCOREPREDICTION
#define H_GROVE_SCOREPREDICTION

#include <orx/base/ORImagePtrTypes.h>

#include "Keypoint3DColourCluster.h"
#include "../util/Array.h"

namespace grove {

//#################### TYPEDEFS ####################

typedef Array<Keypoint3DColourCluster,50> ScorePrediction;

typedef ORUtils::Image<ScorePrediction> ScorePredictionsImage;
typedef boost::shared_ptr<ScorePredictionsImage> ScorePredictionsImage_Ptr;
typedef boost::shared_ptr<const ScorePredictionsImage> ScorePredictionsImage_CPtr;

typedef ORUtils::MemoryBlock<ScorePrediction> ScorePredictionsMemoryBlock;
typedef boost::shared_ptr<ScorePredictionsMemoryBlock> ScorePredictionsMemoryBlock_Ptr;
typedef boost::shared_ptr<const ScorePredictionsMemoryBlock> ScorePredictionsMemoryBlock_CPtr;

//#################### FUNCTIONS ####################

/**
 * \brief Attempts to find the index of the mode in the specified SCoRe forest prediction whose Mahalanobis distance to the specified 3D point is smallest.
 *
 * \note  This version of the function also returns the energy associated with the mode found (if any).
 *
 * \param pt                The 3D point (in world coordinates).
 * \param prediction        The SCoRe forest prediction.
 * \param closestModeEnergy A location in which to store the energy associated with the closest mode (if any).
 * \return                  The index of the closest mode in the prediction, if any, or -1 if the prediction does not contain any modes.
 */
_CPU_AND_GPU_CODE_
inline int find_closest_mode(const Vector3f& pt, const ScorePrediction& prediction, float& closestModeEnergy)
{
  const float exponent = powf(2.0f * static_cast<float>(M_PI), 3);

  // Initialise the closest mode index. If the prediction has modes, we set this to 0 to force the selection of a mode
  // regardless of what happens in the loop. This circumvents the numerical problems that can arise if every mode has
  // a large inverse covariance and a small determinant, leading to a very small covariance.
  int closestModeIdx = prediction.size > 0 ? 0 : -1;

  closestModeEnergy = 0.0f;

  // For each mode in the prediction:
  for(int i = 0; i < prediction.size; ++i)
  {
    // Compute an energy for the mode based on its Mahalanobis distance to the 3D point. Note that we use the textbook implementation
    // of Mahalanobis distance as opposed to the one in Helpers::MahalanobisSquared3x3 from the ScoreForests code (which seems wrong).
    const Keypoint3DColourCluster& currentMode = prediction.elts[i];

    const Vector3f diff = pt - currentMode.position;
    const float mahalanobisSq = dot(diff, currentMode.positionInvCovariance * diff);

    const float descriptiveStatistics = expf(-0.5f * mahalanobisSq);
    const float normalization = 1.0f / sqrtf(currentMode.determinant * exponent);
    const float evalGaussian = normalization * descriptiveStatistics;

    const float nbPts = static_cast<float>(currentMode.nbInliers);
    const float energy = nbPts * evalGaussian;

    // If the point is "closer" to the centre of the anisotropic Gaussian associated with this mode, update the closest mode index.
    if(energy > closestModeEnergy)
    {
      closestModeIdx = i;
      closestModeEnergy = energy;
    }
  }

  return closestModeIdx;
}

/**
 * \brief Attempts to find the index of the mode in the specified SCoRe forest prediction whose Mahalanobis distance to the specified 3D point is smallest.
 *
 * \param pt          The 3D point (in world coordinates).
 * \param prediction  The SCoRe forest prediction.
 * \return            The index of the closest mode in the prediction, if any, or -1 if the prediction does not contain any modes.
 */
_CPU_AND_GPU_CODE_
inline int find_closest_mode(const Vector3f& pt, const ScorePrediction& prediction)
{
  float energy;
  return find_closest_mode(pt, prediction, energy);
}

}

#endif
