/**
 * grove: ScorePrediction.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCOREPREDICTION
#define H_GROVE_SCOREPREDICTION

#include <itmx/base/ITMImagePtrTypes.h>

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
 * \param pt          The 3D point (in world coordinates).
 * \param prediction  The SCoRe forest prediction.
 * \param maxEnergy   A location in which to store the energy associated with the closest mode (if any).
 * \return            The index of the closest mode in the prediction, if any, or -1 if the prediction does not contain any modes.
 */
_CPU_AND_GPU_CODE_
inline int find_closest_mode(const Vector3f& pt, const ScorePrediction& prediction, float& maxEnergy)
{
  const float exponent = powf(2.0f * static_cast<float>(M_PI), 3);

  // Set to -1 only if there are no modes, we set it to 0 otherwise, to force the selection of a mode
  // in the case of a very small covariance that otherwise would cause numerical problems (large inverse covariance and small determinant).
  int argmax = prediction.size > 0 ? 0 : -1;
  maxEnergy = 0.0f;

  // Iterate over all the modal clusters stored in the struct.
  for(int m = 0; m < prediction.size; ++m)
  {
    const Keypoint3DColourCluster& currentMode = prediction.elts[m];

    const Vector3f diff = pt - currentMode.position;

    // This is the textbook implementation of Mahalanobis distance
    // Helpers::MahalanobisSquared3x3 used in the scoreforests code seems wrong.
    const float mahalanobisSq = dot(diff, currentMode.positionInvCovariance * diff);
    const float descriptiveStatistics = expf(-0.5f * mahalanobisSq);

    const float normalization = 1.0f / sqrtf(currentMode.determinant * exponent);
    const float evalGaussian = normalization * descriptiveStatistics;

    const float nbPts = static_cast<float>(currentMode.nbInliers);
    const float energy = nbPts * evalGaussian;

    // If the point is "closer" to the center of the anisotropic gaussian associated to this mode, store its index.
    if(energy > maxEnergy)
    {
      maxEnergy = energy;
      argmax = m;
    }
  }

  return argmax;
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
